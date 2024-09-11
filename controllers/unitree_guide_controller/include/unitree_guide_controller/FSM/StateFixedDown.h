//
// Created by tlab-uav on 24-9-11.
//

#ifndef STATEFIXEDDOWN_H
#define STATEFIXEDDOWN_H

#include "FSMState.h"

class StateFixedDown final : public FSMState {
public:
    explicit StateFixedDown(CtrlComponent ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;
private:
    double target_pos_[12] = {
        0.0473455, 1.22187, -2.44375, -0.0473455,
        1.22187, -2.44375, 0.0473455, 1.22187,
        -2.44375, -0.0473455, 1.22187, -2.44375
    };

    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double duration_ = 6000; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDDOWN_H
