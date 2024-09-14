//
// Created by tlab-uav on 24-9-11.
//

#include "unitree_guide_controller/FSM/StateFixedDown.h"

#include <cmath>

StateFixedDown::StateFixedDown(CtrlComponent ctrlComp): FSMState(
    FSMStateName::FIXEDDOWN, "fixed down", std::move(ctrlComp)) {
    duration_ = ctrlComp_.frequency_ * 1.2;
}

void StateFixedDown::enter() {
    for (int i = 0; i < 12; i++) {
        start_pos_[i] = ctrlComp_.joint_position_state_interface_[i].get().get_value();
    }
}

void StateFixedDown::run() {
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++) {
        ctrlComp_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        ctrlComp_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrlComp_.joint_effort_command_interface_[i].get().set_value(0);
        ctrlComp_.joint_kp_command_interface_[i].get().set_value(50.0);
        ctrlComp_.joint_kd_command_interface_[i].get().set_value(3.5);
    }
}

void StateFixedDown::exit() {
    percent_ = 0;
}

FSMStateName StateFixedDown::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDDOWN;
    }
    switch (ctrlComp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::FIXEDDOWN;
    }
}
