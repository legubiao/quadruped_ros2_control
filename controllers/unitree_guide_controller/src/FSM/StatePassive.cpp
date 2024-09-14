//
// Created by tlab-uav on 24-9-6.
//

#include "unitree_guide_controller/FSM/StatePassive.h"

#include <iostream>
#include <utility>

StatePassive::StatePassive(CtrlComponent ctrlComp) : FSMState(
    FSMStateName::PASSIVE, "passive", std::move(ctrlComp)) {
}

void StatePassive::enter() {
    for (auto i: ctrlComp_.joint_effort_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrlComp_.joint_position_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrlComp_.joint_velocity_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrlComp_.joint_kp_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrlComp_.joint_kd_command_interface_) {
        i.get().set_value(1);
    }
}

void StatePassive::run() {
}

void StatePassive::exit() {
}

FSMStateName StatePassive::checkChange() {
    if (ctrlComp_.control_inputs_.get().command == 2) {
        return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVE;
}
