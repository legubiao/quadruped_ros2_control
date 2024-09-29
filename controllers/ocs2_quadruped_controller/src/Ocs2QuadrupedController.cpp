//
// Created by tlab-uav on 24-9-24.
//

#include "Ocs2QuadrupedController.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_sqp/SqpMpc.h>
#include <angles/angles.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>

namespace ocs2::legged_robot {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: command_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Ocs2QuadrupedController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto &interface_type: imu_interface_types_) {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        for (const auto &interface_type: foot_force_interface_types_) {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type Ocs2QuadrupedController::update(const rclcpp::Time &time,
                                                                      const rclcpp::Duration &period) {
        // State Estimate
        updateStateEstimation(time, period);

        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(current_observation_);

        // Load the latest MPC policy
        mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        vector_t optimized_state, optimized_input;
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state,
                                           optimized_input, planned_mode);

        // Whole body control
        current_observation_.input = optimized_input;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state_, planned_mode, period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state, legged_interface_->getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input,
                                                               legged_interface_->getCentroidalModelInfo());

        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(current_observation_, optimized_state, optimized_input)) {
            RCLCPP_ERROR(get_node()->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
            for (int i = 0; i < joint_names_.size(); i++) {
                ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_position_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0);
                ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0.0);
                ctrl_comp_.joint_kd_command_interface_[i].get().set_value(0.35);
            }
            return controller_interface::return_type::ERROR;
        }

        for (int i = 0; i < joint_names_.size(); i++) {
            ctrl_comp_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_comp_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0.0);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(1.0);
        }

        observation_publisher_->publish(ros_msg_conversions::createObservationMsg(current_observation_));

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init() {
        // Initialize OCS2
        urdf_file_ = auto_declare<std::string>("urdf_file", urdf_file_);
        task_file_ = auto_declare<std::string>("task_file", task_file_);
        reference_file_ = auto_declare<std::string>("reference_file", reference_file_);
        gait_file_ = auto_declare<std::string>("gait_file", gait_file_);

        // Load verbose parameter from the task file
        verbose_ = false;
        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        // Hardware Parameters
        joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
        command_interface_types_ =
                auto_declare<std::vector<std::string> >("command_interfaces", command_interface_types_);
        state_interface_types_ =
                auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);
        imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
        imu_interface_types_ = auto_declare<std::vector<std::string> >("imu_interfaces", state_interface_types_);
        foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
        foot_force_interface_types_ =
                auto_declare<std::vector<std::string> >("foot_force_interfaces", state_interface_types_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        // Visualization
        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);
        // robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
        //                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // State estimation
        setupStateEstimate();

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(),
                                             legged_interface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_);
        wbc_->loadTasksSetting(task_file_, verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
                // Handle message
                ctrl_comp_.control_inputs_.command = msg->command;
                ctrl_comp_.control_inputs_.lx = msg->lx;
                ctrl_comp_.control_inputs_.ly = msg->ly;
                ctrl_comp_.control_inputs_.rx = msg->rx;
                ctrl_comp_.control_inputs_.ry = msg->ry;
            });

        observation_publisher_ = get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(
            "legged_robot_mpc_observation", 10);

        get_node()->get_parameter("update_rate", ctrl_comp_.frequency_);
        RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_comp_.frequency_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        // clear out vectors in case of restart
        ctrl_comp_.clear();

        // assign command interfaces
        for (auto &interface: command_interfaces_) {
            std::string interface_name = interface.get_interface_name();
            std::cout << "interface_name: " << interface.get_prefix_name() << std::endl;
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos) {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            } else {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            if (interface.get_prefix_name() == imu_name_) {
                ctrl_comp_.imu_state_interface_.emplace_back(interface);
            } else if (interface.get_prefix_name() == foot_force_name_) {
                ctrl_comp_.foot_force_state_interface_.emplace_back(interface);
            } else {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        if (mpc_running_ == false) {
            // Initial state
            current_observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
            updateStateEstimation(get_node()->now(), rclcpp::Duration(0, 200000));
            current_observation_.input.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
            current_observation_.mode = STANCE;

            const TargetTrajectories target_trajectories({current_observation_.time}, {current_observation_.state},
                                                         {current_observation_.input});

            // Set the first observation and command and wait for optimization to finish
            mpc_mrt_interface_->setCurrentObservation(current_observation_);
            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(get_node()->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived()) {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(get_node()->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        return CallbackReturn::SUCCESS;
    }

    void Ocs2QuadrupedController::setupLeggedInterface() {
        legged_interface_ = std::make_shared<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);
    }

    void Ocs2QuadrupedController::setupMpc() {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());
        rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        const std::string robotName = "legged_robot";

        // Todo Handle Gait Receive.
        // Gait receiver
        const auto gait_manager_ptr = std::make_shared<GaitManager>(
            ctrl_comp_, legged_interface_->getSwitchedModelReferenceManagerPtr()->
            getGaitSchedule());
        gait_manager_ptr->init(gait_file_);

        // Todo Here maybe the reason of the nullPointer.
        // ROS ReferenceManager
        const auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
            robotName, legged_interface_->getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(get_node());
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr);
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    }

    void Ocs2QuadrupedController::setupMrt() {
        mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&] {
            while (controller_running_) {
                try {
                    executeAndSleep(
                        [&] {
                            if (mpc_running_) {
                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception &e) {
                    controller_running_ = false;
                    RCLCPP_WARN(get_node()->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(get_node()->get_logger(), "MRT initialized. MPC thread started.");
    }

    void Ocs2QuadrupedController::setupStateEstimate() {
        ctrl_comp_.estimator_ = std::make_shared<KalmanFilterEstimate>(legged_interface_->getPinocchioInterface(),
                                                                       legged_interface_->getCentroidalModelInfo(),
                                                                       *eeKinematicsPtr_, ctrl_comp_, this->get_node());
        dynamic_cast<KalmanFilterEstimate &>(*ctrl_comp_.estimator_).loadSettings(task_file_, verbose_);
        current_observation_.time = 0;
    }

    void Ocs2QuadrupedController::updateStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period) {
        measured_rbd_state_ = ctrl_comp_.estimator_->update(time, period);
        current_observation_.time += period.seconds();
        const scalar_t yaw_last = current_observation_.state(9);
        current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(
                                            yaw_last, current_observation_.state(9));
        current_observation_.mode = ctrl_comp_.estimator_->getMode();
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::legged_robot::Ocs2QuadrupedController, controller_interface::ControllerInterface);
