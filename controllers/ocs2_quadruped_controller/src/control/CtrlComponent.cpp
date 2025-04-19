//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();

        if (!node_->has_parameter("enable_perceptive")) {
            node_->declare_parameter("enable_perceptive", enable_perceptive_);
        }
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();


        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";

        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Init visualizer
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        // Init observation
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;
    }

    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        if (estimator_type == "ground_truth")
        {
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else
        {
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Update State Estimation
        measured_rbd_state_ = estimator_->update(time, period);
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        observation_.mode = estimator_->getMode();

        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_->update(observation_);
            sphereVisualizationPtr_->update(observation_);
        }

        // Compute target trajectory
        target_manager_->update(observation_);
        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    void CtrlComponent::init()
    {
        if (mpc_running_ == false)
        {
            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
        }
        else
        {
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        }

        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

            sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                *dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr(), node_);
        }
    }

    /**
     * Set up the SQP MPC, Gait Manager and Reference Manager
     */
    void CtrlComponent::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule());
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_,
                                                          reference_file_);

        if (enable_perceptive_)
        {
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    void CtrlComponent::setupMrt()
    {
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                }
                catch (const std::exception& e)
                {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }
}
