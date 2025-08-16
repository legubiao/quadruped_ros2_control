//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <controller_common/FSM/BaseFixedStand.h>

class StateFixedStand final : public BaseFixedStand {
public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd);

    FSMStateName checkChange() override;
};


#endif //STATEFIXEDSTAND_H
