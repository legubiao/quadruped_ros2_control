//
// Created by biao on 24-9-10.
//

#include <unitree_guide_controller/FSM/StateFixedStand.h>

StateFixedStand::StateFixedStand(CtrlComponent ctrlComp): FSMState(
    FSMStateName::FIXEDSTAND, "fixed stand", std::move(ctrlComp)) {
}

void StateFixedStand::enter() {
}

void StateFixedStand::run() {
}

void StateFixedStand::exit() {
}

FSMStateName StateFixedStand::checkChange() {
    return FSMStateName::FIXEDSTAND;
}
