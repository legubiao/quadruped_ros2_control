//
// Created by tlab-uav on 24-9-11.
//

#ifndef STATEFIXEDDOWN_H
#define STATEFIXEDDOWN_H

#include <rclcpp/time.hpp>

#include "FSMState.h"

class StateFixedDown final : public FSMState {
public:
    explicit StateFixedDown(CtrlComponent &ctrlComp,
                            const std::vector<double> &target_pos,
                            double kp,
                            double kd
    );

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    double target_pos_[12] = {};
    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};


#endif //STATEFIXEDDOWN_H
