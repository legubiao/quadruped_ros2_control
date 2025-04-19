//
// Created by qiayuan on 2022/7/16.
//
#ifndef LEGGEDINTERFACE_H
#define LEGGEDINTERFACE_H


#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "SwitchedModelReferenceManager.h"

namespace ocs2::legged_robot
{
    class LeggedInterface : public RobotInterface
    {
    public:
        LeggedInterface(const std::string& task_file,
                        const std::string& urdf_file,
                        const std::string& reference_file,
                        bool use_hard_friction_cone_constraint = false);

        ~LeggedInterface() override = default;

        void setupJointNames(const std::vector<std::string>& joint_names,
                             const std::vector<std::string>& foot_names);

        virtual void setupOptimalControlProblem(const std::string& task_file,
                                                const std::string& urdf_file,
                                                const std::string& reference_file,
                                                bool verbose);

        const OptimalControlProblem& getOptimalControlProblem() const override { return *problem_ptr_; }

        const ModelSettings& modelSettings() const { return model_settings_; }
        const mpc::Settings& mpcSettings() const { return mpc_settings_; }
        const sqp::Settings& sqpSettings() { return sqp_settings_; }

        const RolloutBase& getRollout() const { return *rollout_ptr_; }
        PinocchioInterface& getPinocchioInterface() { return *pinocchio_interface_ptr_; }
        const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidal_model_info_; }

        std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const
        {
            return reference_manager_ptr_;
        }

        const Initializer& getInitializer() const override { return *initializer_ptr_; }

        std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override
        {
            return reference_manager_ptr_;
        }

    protected:
        void setupModel(const std::string& task_file, const std::string& urdf_file,
                        const std::string& reference_file);

        virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                           const std::string& referenceFile,
                                           bool verbose);

        std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

        std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile,
                                                            const CentroidalModelInfo& info, bool verbose);

        matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

        static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(
            const std::string& taskFile, bool verbose);

        std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex,
                                                                        scalar_t frictionCoefficient);

        std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex,
                                                                      scalar_t frictionCoefficient,
                                                                      const RelaxedBarrierPenalty::Config&
                                                                      barrierPenaltyConfig);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& foot_names,
                                                                            const std::string& model_name);

        std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(
            const EndEffectorKinematics<scalar_t>& end_effector_kinematics,
            size_t contact_point_index);

        std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                              const std::string& taskFile,
                                                              const std::string& urdf_file,
                                                              const std::string& prefix, bool verbose);

        ModelSettings model_settings_;
        mpc::Settings mpc_settings_;
        sqp::Settings sqp_settings_;
        const bool use_hard_friction_cone_constraint_;

        std::unique_ptr<PinocchioInterface> pinocchio_interface_ptr_;
        CentroidalModelInfo centroidal_model_info_;
        std::unique_ptr<PinocchioGeometryInterface> geometry_interface_ptr_;

        std::unique_ptr<OptimalControlProblem> problem_ptr_;
        std::shared_ptr<SwitchedModelReferenceManager> reference_manager_ptr_;

        rollout::Settings rollout_settings_;
        std::unique_ptr<RolloutBase> rollout_ptr_;
        std::unique_ptr<Initializer> initializer_ptr_;

        vector_t initial_state_;
    };
} // namespace legged
#endif // LEGGEDINTERFACE_H
