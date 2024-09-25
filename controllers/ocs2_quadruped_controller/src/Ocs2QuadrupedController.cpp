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
#include <ocs2_sqp/SqpMpc.h>
#include <angles/angles.h>

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
        const vector_t rbd_state = ctrl_comp_.estimator_->update(time, period);
        currentObservation_.time += period.seconds();
        const scalar_t yaw_last = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(rbd_state);
        currentObservation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, currentObservation_.state(9));
        currentObservation_.mode = stateEstimate_->getMode();

        // Update the current state of the system
        mpcMrtInterface_->setCurrentObservation(currentObservation_);

        // Load the latest MPC policy
        mpcMrtInterface_->updatePolicy();

        // Evaluate the current policy
        vector_t optimizedState, optimizedInput;
        size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
        mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

        // Whole body control
        currentObservation_.input = optimizedInput;

        wbcTimer_.startTimer();
        vector_t x = wbc_->update(optimizedState, optimizedInput, rbd_state, plannedMode, period.seconds());
        wbcTimer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t posDes = centroidal_model::getJointAngles(optimizedState, legged_interface_->getCentroidalModelInfo());
        vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, legged_interface_->getCentroidalModelInfo());

        // Safety check, if failed, stop the controller
        if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
            RCLCPP_ERROR(get_node()->get_logger(), "[Legged Controller] Safety check failed, stopping the controller.");
        }

        for (int i = 0; i < joint_names_.size(); i++) {
            ctrl_comp_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_comp_.joint_position_command_interface_[i].get().set_value(posDes(i));
            ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(velDes(i));
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0.0);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(3.0);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init() {
        // Initialize OCS2
        urdf_file_ = auto_declare<std::string>("urdf_file", urdf_file_);
        task_file_ = auto_declare<std::string>("task_file", task_file_);
        reference_file_ = auto_declare<std::string>("reference_file", reference_file_);

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
        CentroidalModelPinocchioMapping pinocchioMapping(legged_interface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchioMapping,
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
        safetyChecker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

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
        rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                          legged_interface_->getCentroidalModelInfo());

        const std::string robotName = "legged_robot";

        // Todo Handle Gait Receive.
        // Gait receiver
        // auto gaitReceiverPtr =
        //         std::make_shared<GaitReceiver>(this->get_node(),
        //                                        legged_interface_->getSwitchedModelReferenceManagerPtr()->
        //                                        getGaitSchedule(), robotName);

        // ROS ReferenceManager
        // auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
        //     robotName, legged_interface_->getReferenceManagerPtr());
        // rosReferenceManagerPtr->subscribe(this->get_node());
        // mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
        // mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
        // observationPublisher_ = nh.advertise<ocs2_msgs::msg::mpc_observation>(robotName + "_mpc_observation", 1);
    }

    void Ocs2QuadrupedController::setupMrt() {
        mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpcMrtInterface_->initRollout(&legged_interface_->getRollout());
        mpcTimer_.reset();

        controllerRunning_ = true;
        mpcThread_ = std::thread([&] {
            while (controllerRunning_) {
                try {
                    executeAndSleep(
                        [&] {
                            if (mpcRunning_) {
                                mpcTimer_.startTimer();
                                mpcMrtInterface_->advanceMpc();
                                mpcTimer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception &e) {
                    controllerRunning_ = false;
                    RCLCPP_WARN(get_node()->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpcThread_);
    }

    void Ocs2QuadrupedController::setupStateEstimate() {
        ctrl_comp_.estimator_ = std::make_shared<KalmanFilterEstimate>(legged_interface_->getPinocchioInterface(),
                                                                       legged_interface_->getCentroidalModelInfo(),
                                                                       *eeKinematicsPtr_, ctrl_comp_, this->get_node());
        dynamic_cast<KalmanFilterEstimate &>(*ctrl_comp_.estimator_).loadSettings(task_file_, verbose_);
        currentObservation_.time = 0;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::legged_robot::Ocs2QuadrupedController, controller_interface::ControllerInterface);