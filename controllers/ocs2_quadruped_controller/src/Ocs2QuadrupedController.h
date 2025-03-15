//
// Created by tlab-uav on 24-9-24.
//

#ifndef OCS2QUADRUPEDCONTROLLER_H
#define OCS2QUADRUPEDCONTROLLER_H

#include <controller_common/FSM/StatePassive.h>
#include <controller_interface/controller_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <ocs2_quadruped_controller/FSM/StateOCS2.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>

namespace ocs2::legged_robot {
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateOCS2> fixedDown;
    };

    class Ocs2QuadrupedController final : public controller_interface::ControllerInterface {
    public:
        Ocs2QuadrupedController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;


        controller_interface::CallbackReturn on_init() override;


        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:

        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        CtrlInterfaces ctrl_interfaces_;
        std::shared_ptr<CtrlComponent> ctrl_comp_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        std::string command_prefix_;

        // IMU Sensor
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;

        // Foot Force Sensor
        std::string foot_force_name_;
        std::vector<std::string> foot_force_interface_types_;

        // Odometer Sensor (Ground Truth)
        std::string estimator_type_ = "linear_kalman";
        std::string odom_name_;
        std::vector<std::string> odom_interface_types_;

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
    };
}

#endif //OCS2QUADRUPEDCONTROLLER_H
