//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include "FSMState.h"

class StateFixedStand final : public FSMState {
public:
    explicit StateFixedStand(CtrlComponent &ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    double target_pos_[12] = {
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763
    };

    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDSTAND_H
