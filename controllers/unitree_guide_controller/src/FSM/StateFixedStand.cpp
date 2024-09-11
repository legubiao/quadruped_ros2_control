//
// Created by biao on 24-9-10.
//

#include <unitree_guide_controller/FSM/StateFixedStand.h>

StateFixedStand::StateFixedStand(CtrlComponent ctrlComp): FSMState(
    FSMStateName::FIXEDSTAND, "fixed stand", std::move(ctrlComp)) {
}

void StateFixedStand::enter() {
    for (int i = 0; i < 12; i++) {
        ctrlComp_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
    }
}

void StateFixedStand::run() {
}

void StateFixedStand::exit() {
}

FSMStateName StateFixedStand::checkChange() {
    if (ctrlComp_.control_inputs_.get().command == 1) {
        return FSMStateName::PASSIVE;
    }
    return FSMStateName::FIXEDSTAND;
}
