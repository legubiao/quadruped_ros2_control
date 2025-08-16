//
// Created by biao on 24-9-10.
//

#include "basic_quadruped_controller/FSM/StateFixedStand.h"
#include "basic_quadruped_controller/control/CtrlComponent.h"

StateFixedStand::StateFixedStand(CtrlInterfaces& ctrl_interfaces, 
                                 CtrlComponent& ctrl_component,
                                 const std::vector<double>& target_pos,
                                 const double kp,
                                 const double kd)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd),
      ctrl_component_(ctrl_component)
{
}

void StateFixedStand::enter() {
    // Call parent enter method
    BaseFixedStand::enter();
}

FSMStateName StateFixedStand::checkChange()
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
    case 3:
        return FSMStateName::FREESTAND;
    case 4:
        return FSMStateName::TROTTING;
    case 5:
        return FSMStateName::BALANCETEST;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}
