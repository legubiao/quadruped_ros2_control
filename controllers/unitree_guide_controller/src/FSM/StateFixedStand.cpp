//
// Created by biao on 24-9-10.
//

#include <cmath>
#include <iostream>
#include <unitree_guide_controller/FSM/StateFixedStand.h>

StateFixedStand::StateFixedStand(CtrlComponent ctrlComp): FSMState(
    FSMStateName::FIXEDSTAND, "fixed stand", std::move(ctrlComp)) {
}

void StateFixedStand::enter() {
    for (int i = 0; i < 12; i++) {
        start_pos_[i] = ctrlComp_.joint_position_state_interface_[i].get().get_value();
    }
}

void StateFixedStand::run() {
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++) {
        ctrlComp_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
        ctrlComp_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrlComp_.joint_effort_command_interface_[i].get().set_value(0);
        ctrlComp_.joint_kp_command_interface_[i].get().set_value(
            phase * 60.0 + (1 - phase) * 20.0);
        ctrlComp_.joint_kd_command_interface_[i].get().set_value(3.5);
    }
}

void StateFixedStand::exit() {
    percent_ = 0;
}

FSMStateName StateFixedStand::checkChange() {
    if (percent_ < 2) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrlComp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 3:
            return FSMStateName::FREESTAND;
        case 9:
            return FSMStateName::SWINGTEST;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
