//
// Created by biao on 24-9-10.
//

#include "controller_common/FSM/BaseFixedStand.h"

#include <cmath>

BaseFixedStand::BaseFixedStand(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "fixed stand", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void BaseFixedStand::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void BaseFixedStand::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }
}

void BaseFixedStand::exit()
{
    percent_ = 0;
}

FSMStateName BaseFixedStand::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}
