//
// Created by biao on 24-9-12.
//

#include <unitree_guide_controller/FSM/StateSwingTest.h>

StateSwingTest::StateSwingTest(CtrlComponent ctrlComp): FSMState(
    FSMStateName::SWINGTEST, "swing test", std::move(ctrlComp)) {
    _xMin = -0.15;
    _xMax = 0.10;
    _yMin = -0.15;
    _yMax = 0.15;
    _zMin = -0.05;
    _zMax = 0.20;
}

void StateSwingTest::enter() {
}

void StateSwingTest::run() {
}

void StateSwingTest::exit() {
}

FSMStateName StateSwingTest::checkChange() {
    switch (ctrlComp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::SWINGTEST;
    }
}
