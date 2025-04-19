//
// Created by qiayuan on 2022/7/16.
//

#include "ocs2_quadruped_controller/interface/LeggedInterface.h"

#include <memory>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h"
#include "ocs2_quadruped_controller/interface/constraint/FrictionConeConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/LeggedSelfCollisionConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_quadruped_controller/interface/constraint/ZeroForceConstraint.h"
#include "ocs2_quadruped_controller/interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_quadruped_controller/interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "ocs2_quadruped_controller/interface/initialization/LeggedRobotInitializer.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::legged_robot
{
    LeggedInterface::LeggedInterface(const std::string& task_file,
                                     const std::string& urdf_file,
                                     const std::string& reference_file,
                                     const bool use_hard_friction_cone_constraint)
        : use_hard_friction_cone_constraint_(use_hard_friction_cone_constraint)
    {
        // check that task file exists
        if (const boost::filesystem::path task_file_path(task_file); exists(task_file_path))
        {
            std::cerr << "[LeggedInterface] Loading task file: " << task_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument("[LeggedInterface] Task file not found: " + task_file_path.string());
        }

        // check that urdf file exists
        if (const boost::filesystem::path urdf_file_path(urdf_file); exists(urdf_file_path))
        {
            std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdf_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdf_file_path.string());
        }

        // check that targetCommand file exists
        if (const boost::filesystem::path reference_file_path(reference_file); exists(reference_file_path))
        {
            std::cerr << "[LeggedInterface] Loading target command settings from: " << reference_file_path << std::endl;
        }
        else
        {
            throw std::invalid_argument(
                "[LeggedInterface] targetCommand file not found: " + reference_file_path.string());
        }

        bool verbose = false;
        loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);

        // load setting from loading file
        model_settings_ = loadModelSettings(task_file, "model_settings", verbose);

        // If modelFolderCppAd doesn't start with "/", prepend home directory
        if (!model_settings_.modelFolderCppAd.empty() && model_settings_.modelFolderCppAd.front() != '/')
        {
            if (const char* home_dir = std::getenv("HOME"); home_dir != nullptr)
            {
                model_settings_.modelFolderCppAd = std::string(home_dir) + "/" + model_settings_.modelFolderCppAd;
            }
        }

        mpc_settings_ = mpc::loadSettings(task_file, "mpc", verbose);
        sqp_settings_ = sqp::loadSettings(task_file, "sqp", verbose);
        rollout_settings_ = rollout::loadSettings(task_file, "rollout", verbose);
    }


    void LeggedInterface::setupJointNames(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& foot_names)
    {
        model_settings_.jointNames = joint_names;
        model_settings_.contactNames3DoF = foot_names;
    }

    void LeggedInterface::setupOptimalControlProblem(const std::string& task_file,
                                                     const std::string& urdf_file,
                                                     const std::string& reference_file,
                                                     const bool verbose)
    {
        setupModel(task_file, urdf_file, reference_file);

        // Initial state
        initial_state_.setZero(centroidal_model_info_.stateDim);
        loadData::loadEigenMatrix(task_file, "initialState", initial_state_);

        setupReferenceManager(task_file, urdf_file, reference_file, verbose);

        // Optimal control problem
        problem_ptr_ = std::make_unique<OptimalControlProblem>();

        // Dynamics
        std::unique_ptr<SystemDynamicsBase> dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(
            *pinocchio_interface_ptr_, centroidal_model_info_, "dynamics",
            model_settings_);
        problem_ptr_->dynamicsPtr = std::move(dynamicsPtr);

        // Cost terms
        problem_ptr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(task_file, centroidal_model_info_, verbose));

        // Constraint terms
        // friction cone settings
        scalar_t frictionCoefficient = 0.7;
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(task_file, verbose);

        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = model_settings_.contactNames3DoF[i];
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr =
                getEeKinematicsPtr({footName}, footName);

            if (use_hard_friction_cone_constraint_)
            {
                problem_ptr_->inequalityConstraintPtr->add(footName + "_frictionCone",
                                                           getFrictionConeConstraint(i, frictionCoefficient));
            }
            else
            {
                problem_ptr_->softConstraintPtr->add(footName + "_frictionCone",
                                                     getFrictionConeSoftConstraint(
                                                         i, frictionCoefficient, barrierPenaltyConfig));
            }
            problem_ptr_->equalityConstraintPtr->add(footName + "_zeroForce", std::make_unique<ZeroForceConstraint>(
                                                         *reference_manager_ptr_, i, centroidal_model_info_));
            problem_ptr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                                     getZeroVelocityConstraint(*eeKinematicsPtr, i));
            problem_ptr_->equalityConstraintPtr->add(
                footName + "_normalVelocity",
                std::make_unique<NormalVelocityConstraintCppAd>(*reference_manager_ptr_, *eeKinematicsPtr, i));
        }

        // Self-collision avoidance constraint
        problem_ptr_->stateSoftConstraintPtr->add("selfCollision",
                                                  getSelfCollisionConstraint(
                                                      *pinocchio_interface_ptr_, task_file, urdf_file, "selfCollision",
                                                      verbose));

        // Setup Problem PreComputation
        problem_ptr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_);

        // Rollout
        rollout_ptr_ = std::make_unique<TimeTriggeredRollout>(*problem_ptr_->dynamicsPtr, rollout_settings_);

        // Initialization
        constexpr bool extend_normalized_momentum = true;
        initializer_ptr_ = std::make_unique<LeggedRobotInitializer>(centroidal_model_info_, *reference_manager_ptr_,
                                                                    extend_normalized_momentum);
    }


    void LeggedInterface::setupModel(const std::string& task_file, const std::string& urdf_file,
                                     const std::string& reference_file)
    {
        // PinocchioInterface
        pinocchio_interface_ptr_ =
            std::make_unique<PinocchioInterface>(
                centroidal_model::createPinocchioInterface(urdf_file, model_settings_.jointNames));

        // CentroidModelInfo
        centroidal_model_info_ = centroidal_model::createCentroidalModelInfo(
            *pinocchio_interface_ptr_, centroidal_model::loadCentroidalType(task_file),
            centroidal_model::loadDefaultJointState(pinocchio_interface_ptr_->getModel().nq - 6, reference_file),
            model_settings_.contactNames3DoF,
            model_settings_.contactNames6DoF);
    }


    void LeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                                const std::string& referenceFile,
                                                const bool verbose)
    {
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
        reference_manager_ptr_ =
            std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                            std::move(swingTrajectoryPlanner));
    }


    std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const
    {
        const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
        const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

        const auto defaultGait = [defaultModeSequenceTemplate]
        {
            Gait gait{};
            gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
            // Events: from time -> phase
            std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                          defaultModeSequenceTemplate.switchingTimes.end() - 1,
                          [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
            // Modes:
            gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
            return gait;
        }();

        // display
        if (verbose)
        {
            std::cerr << "\n#### Modes Schedule: ";
            std::cerr << "\n#### =============================================================================\n";
            std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
            std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
            std::cerr << "#### =============================================================================\n";
        }

        return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate,
                                              model_settings_.phaseTransitionStanceTime);
    }


    matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
    {
        const size_t totalContactDim = 3 * info.numThreeDofContacts;

        const auto& model = pinocchio_interface_ptr_->getModel();
        auto& data = pinocchio_interface_ptr_->getData();
        const auto q = centroidal_model::getGeneralizedCoordinates(initial_state_, centroidal_model_info_);
        computeJointJacobians(model, data, q);
        updateFramePlacements(model, data);

        matrix_t base2feetJac(totalContactDim, info.actuatedDofNum);
        for (size_t i = 0; i < info.numThreeDofContacts; i++)
        {
            matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
            getFrameJacobian(model, data, model.getBodyId(model_settings_.contactNames3DoF[i]),
                             pinocchio::LOCAL_WORLD_ALIGNED, jac);
            base2feetJac.block(3 * i, 0, 3, info.actuatedDofNum) = jac.block(0, 6, 3, info.actuatedDofNum);
        }

        matrix_t rTaskspace(info.inputDim, info.inputDim);
        loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
        matrix_t r = rTaskspace;
        // Joint velocities
        r.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) =
            base2feetJac.transpose() * rTaskspace.block(totalContactDim, totalContactDim, info.actuatedDofNum,
                                                        info.actuatedDofNum) *
            base2feetJac;
        return r;
    }


    std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(
        const std::string& taskFile, const CentroidalModelInfo& info,
        bool verbose)
    {
        matrix_t Q(info.stateDim, info.stateDim);
        loadData::loadEigenMatrix(taskFile, "Q", Q);
        matrix_t R = initializeInputCostWeight(taskFile, info);

        if (verbose)
        {
            std::cerr << "\n #### Base Tracking Cost Coefficients: ";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "Q:\n" << Q << "\n";
            std::cerr << "R:\n" << R << "\n";
            std::cerr << " #### =============================================================================\n";
        }

        return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info,
                                                                    *reference_manager_ptr_);
    }


    std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedInterface::loadFrictionConeSettings(
        const std::string& taskFile, bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        const std::string prefix = "frictionConeSoftConstraint.";

        scalar_t frictionCoefficient = 1.0;
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        if (verbose)
        {
            std::cerr << "\n #### Friction Cone Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }

        return {frictionCoefficient, barrierPenaltyConfig};
    }


    std::unique_ptr<StateInputConstraint> LeggedInterface::getFrictionConeConstraint(
        size_t contactPointIndex, scalar_t frictionCoefficient)
    {
        FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
        return std::make_unique<FrictionConeConstraint>(*reference_manager_ptr_, frictionConeConConfig,
                                                        contactPointIndex,
                                                        centroidal_model_info_);
    }


    std::unique_ptr<StateInputCost> LeggedInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex, scalar_t frictionCoefficient,
        const RelaxedBarrierPenalty::Config& barrierPenaltyConfig)
    {
        return std::make_unique<StateInputSoftConstraint>(
            getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
            std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }


    std::unique_ptr<EndEffectorKinematics<scalar_t>> LeggedInterface::getEeKinematicsPtr(
        const std::vector<std::string>& foot_names,
        const std::string& model_name)
    {
        const auto infoCppAd = centroidal_model_info_.toCppAd();
        const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
        auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state,
                                                   PinocchioInterfaceCppAd& pinocchioInterfaceAd)
        {
            const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
            updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
        };
        std::unique_ptr<EndEffectorKinematics<scalar_t>> end_effector_kinematics = std::make_unique<
            PinocchioEndEffectorKinematicsCppAd>(
            *pinocchio_interface_ptr_, pinocchioMappingCppAd,
            foot_names,
            centroidal_model_info_.stateDim,
            centroidal_model_info_.inputDim,
            velocityUpdateCallback, model_name,
            model_settings_.modelFolderCppAd,
            model_settings_.recompileLibrariesCppAd,
            model_settings_.verboseCppAd);

        return end_effector_kinematics;
    }


    std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
        const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
        const size_t contact_point_index)
    {
        auto eeZeroVelConConfig = [](scalar_t positionErrorGain)
        {
            EndEffectorLinearConstraint::Config config;
            config.b.setZero(3);
            config.Av.setIdentity(3, 3);
            if (!numerics::almost_eq(positionErrorGain, 0.0))
            {
                config.Ax.setZero(3, 3);
                config.Ax(2, 2) = positionErrorGain;
            }
            return config;
        };
        return std::make_unique<ZeroVelocityConstraintCppAd>(
            *reference_manager_ptr_, end_effector_kinematics, contact_point_index,
            eeZeroVelConConfig(model_settings_.positionErrorGain));
    }


    std::unique_ptr<StateCost> LeggedInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                           const std::string& taskFile,
                                                                           const std::string& urdf_file,
                                                                           const std::string& prefix,
                                                                           bool verbose)
    {
        std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
        std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
        scalar_t mu = 1e-2;
        scalar_t delta = 1e-3;
        scalar_t minimumDistance = 0.0;

        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        if (verbose)
        {
            std::cerr << "\n #### SelfCollision Settings: ";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
        loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
        loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
        loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

        geometry_interface_ptr_ = std::make_unique<PinocchioGeometryInterface>(
            pinocchioInterface, urdf_file, collisionLinkPairs, collisionObjectPairs);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
            const size_t numCollisionPairs = geometry_interface_ptr_->getNumCollisionPairs();
            std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
        }

        std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
            CentroidalModelPinocchioMapping(centroidal_model_info_), *geometry_interface_ptr_, minimumDistance);

        auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }
} // namespace legged
