//
// Created by tlab-uav on 24-9-6.
//

#ifndef QUADRUPEDCONTROLLER_H
#define QUADRUPEDCONTROLLER_H

#include "controller_interface/controller_interface.hpp"
#include <std_msgs/msg/string.hpp>

namespace quadruped_ros2_control {
    class QuadrupedController final : public controller_interface::ControllerInterface {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        QuadrupedController() = default;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_command_subscriber_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
        joint_effort_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_effort_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_velocity_state_interface_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &joint_effort_command_interface_}
        };

        bool stand_ = false;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &joint_position_state_interface_},
            {"effort", &joint_effort_state_interface_},
            {"velocity", &joint_velocity_state_interface_}
        };
    };
}


#endif //QUADRUPEDCONTROLLER_H
