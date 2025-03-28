//
// Created by tlab-uav on 25-2-27.
//

#include "controller_common/FSM/StatePassive.h"

#include <iostream>

StatePassive::StatePassive(CtrlInterfaces& ctrl_interfaces) : FSMState(
    FSMStateName::PASSIVE, "passive", ctrl_interfaces)
{
}

void StatePassive::enter()
{
    for (auto i : ctrl_interfaces_.joint_torque_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_position_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_velocity_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_kp_command_interface_)
    {
        i.get().set_value(0);
    }
    for (auto i : ctrl_interfaces_.joint_kd_command_interface_)
    {
        i.get().set_value(1);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StatePassive::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
}

void StatePassive::exit()
{
}

FSMStateName StatePassive::checkChange()
{
    if (ctrl_interfaces_.control_inputs_.command == 2)
    {
        return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVE;
}
