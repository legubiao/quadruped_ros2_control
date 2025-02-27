//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <rclcpp/rate.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/control/TargetManager.h>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot {
    StateOCS2::StateOCS2(CtrlInterfaces &ctrl_interfaces,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                         const std::string &package_share_directory,
                         const std::vector<std::string> &joint_names,
                         const std::vector<std::string> &feet_names,
                         double default_kp,
                         double default_kd)
        : FSMState(
              FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          node_(node),
          joint_names_(joint_names),
          feet_names_(feet_names),
          default_kp_(default_kp),
          default_kd_(default_kd) {
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";

        // Load verbose parameter from the task file
        verbose_ = false;
        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        // Visualization
        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        visualizer_ = std::make_shared<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
            node_);

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));
        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(),
                                             legged_interface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_);
        wbc_->loadTasksSetting(task_file_, verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

        observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            "legged_robot_mpc_observation", 10);
    }

    void StateOCS2::enter() {
        if (mpc_running_ == false) {
            // Initial state
            observation_.state.setZero(
                static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
            updateStateEstimation(rclcpp::Duration(0, 1 / ctrl_interfaces_.frequency_ * 1000000000));
            observation_.input.setZero(
                static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
            observation_.mode = STANCE;

            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived()) {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void StateOCS2::run(const rclcpp::Time &time,
                        const rclcpp::Duration &period) {
        if (mpc_running_ == false) {
            return;
        }

        // State Estimate
        updateStateEstimation(period);

        // Compute target trajectory
        target_manager_->update(observation_);

        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);

        // Load the latest MPC policy
        mpc_mrt_interface_->updatePolicy();
        mpc_need_updated_ = true;

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        mpc_mrt_interface_->evaluatePolicy(observation_.time, observation_.state,
                                           optimized_state_,
                                           optimized_input_, planned_mode);

        // Whole body control
        observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, measured_rbd_state_, planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            legged_interface_->getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                legged_interface_->getCentroidalModelInfo());

        for (int i = 0; i < joint_names_.size(); i++) {
            std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        visualizer_->update(observation_, mpc_mrt_interface_->getPolicy(),
                            mpc_mrt_interface_->getCommand());

        observation_publisher_->publish(ros_msg_conversions::createObservationMsg(observation_));
    }

    void StateOCS2::exit() {
        mpc_running_ = false;
        mpc_thread_.join();
        RCLCPP_INFO(node_->get_logger(), "MRT thread stopped.");
    }

    FSMStateName StateOCS2::checkChange() {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(observation_, optimized_state_, optimized_input_)) {
            RCLCPP_ERROR(node_->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
            for (int i = 0; i < joint_names_.size(); i++) {
                std::ignore = ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
                std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(0);
                std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
                std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(0.0);
                std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(0.35);
            }
            return FSMStateName::PASSIVE;
        }
        return FSMStateName::OCS2;
    }

    void StateOCS2::setupStateEstimate(const std::string &estimator_type) {
        if (estimator_type == "ground_truth") {
            estimator_ = std::make_shared<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        } else if (estimator_type == "linear_kalman") {
            estimator_ = std::make_shared<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *eeKinematicsPtr_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate &>(*estimator_).loadSettings(task_file_, verbose_);
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        } else {
            estimator_ = std::make_shared<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void StateOCS2::updateStateEstimation(const rclcpp::Duration &period) {
        measured_rbd_state_ = estimator_->update(node_->now(), period);
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
                                    yaw_last, observation_.state(9));
        // ctrl_comp_.observation_.mode = ctrl_comp_.estimator_->getMode();
    }

    void StateOCS2::setupMrt() {
        mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&] {
            while (controller_running_) {
                try {
                    executeAndSleep(
                        [&] {
                            if (mpc_running_ && mpc_need_updated_) {
                                mpc_need_updated_ = false;
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception &e) {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }

    void StateOCS2::setupLeggedInterface() {
        legged_interface_ = std::make_shared<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);
    }

    void StateOCS2::setupMpc() {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());
        rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Initialize the reference manager
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_interfaces_, legged_interface_->getSwitchedModelReferenceManagerPtr()->
            getGaitSchedule());
        gait_manager_ptr->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        target_manager_ = std::make_shared<TargetManager>(ctrl_interfaces_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_, reference_file_);
    }
}
