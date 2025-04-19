//
// Created by tlab-uav on 24-10-4.
//

#ifndef LEGGEDGYMCONTROLLER_H
#define LEGGEDGYMCONTROLLER_H
#include <controller_interface/controller_interface.hpp>
#include <rl_quadruped_controller/FSM/StateRL.h>
#include <std_msgs/msg/string.hpp>

#include "rl_quadruped_controller/control/CtrlComponent.h"
#include "controller_common/FSM/StateFixedDown.h"
#include "rl_quadruped_controller/FSM/StateFixedStand.h"
#include "controller_common//FSM/StatePassive.h"

namespace rl_quadruped_controller
{
    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateFixedDown> fixedDown;
        std::shared_ptr<StateFixedStand> fixedStand;
        std::shared_ptr<StateRL> rl;
    };

    class LeggedGymController final : public controller_interface::ControllerInterface
    {
    public:
        LeggedGymController() = default;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        CtrlComponent ctrl_component_;
        CtrlInterfaces ctrl_interfaces_;
        std::vector<std::string> joint_names_;
        std::string base_name_ = "base";
        std::vector<std::string> feet_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string command_prefix_;

        // IMU Sensor
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;
        // Foot Force Sensor
        std::string foot_force_name_;
        std::vector<std::string> foot_force_interface_types_;

        // FR FL RR RL
        std::vector<double> stand_pos_ = {
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3,
            0.0, 0.67, -1.3
        };

        std::vector<double> down_pos_ = {
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4,
            0.0, 1.3, -2.4
        };

        double stand_kp_ = 80.0;
        double stand_kd_ = 3.5;
        double feet_force_threshold_ = 20.0;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"effort", &ctrl_interfaces_.joint_effort_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
    };
}
#endif //LEGGEDGYMCONTROLLER_H
