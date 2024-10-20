//
// Created by tlab-uav on 24-9-19.
//

#ifndef LEGPDCONTROLLER_H
#define LEGPDCONTROLLER_H
#include <controller_interface/chainable_controller_interface.hpp>
#include "realtime_tools/realtime_buffer.h"
#include <controller_interface/controller_interface.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"


namespace leg_pd_controller {
    using DataType = std_msgs::msg::Float64MultiArray;
    class LegPdController final : public controller_interface::ChainableControllerInterface {
    public:
        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        bool on_set_chained_mode(bool chained_mode) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        #ifdef ROS2_CONTROL_VERSION_LT_3
        controller_interface::return_type update_reference_from_subscribers() override;
        #else
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period);
        #endif



        std::vector<double> joint_effort_command_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_kp_command_;
        std::vector<double> joint_kd_command_;

        std::vector<std::string> joint_names_;

        std::vector<std::string> state_interface_types_;
        std::vector<std::string> reference_interface_types_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>> rt_buffer_ptr_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
        joint_effort_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
        joint_velocity_state_interface_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &joint_position_state_interface_},
            {"velocity", &joint_velocity_state_interface_}
        };
    };
}


#endif //LEGPDCONTROLLER_H
