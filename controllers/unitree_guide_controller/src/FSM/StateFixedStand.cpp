//
// Created by biao on 24-9-10.
//

#include "unitree_guide_controller/FSM/StateFixedStand.h"

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd) {
}

FSMStateName StateFixedStand::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDDOWN;
        case 3:
            return FSMStateName::FREESTAND;
        case 4:
            return FSMStateName::TROTTING;
        case 5:
            return FSMStateName::SWINGTEST;
        case 6:
            return FSMStateName::BALANCETEST;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
