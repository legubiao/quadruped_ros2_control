//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <controller_common/FSM/BaseFixedStand.h>

struct CtrlComponent;

class StateFixedStand final : public BaseFixedStand {
public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd);

    void enter() override;

    FSMStateName checkChange() override;

private:
    CtrlComponent& ctrl_component_;
};


#endif //STATEFIXEDSTAND_H
