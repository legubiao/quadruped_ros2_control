//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <rclcpp/time.hpp>

#include "FSMState.h"

class StateFixedStand final : public FSMState {
public:
    explicit StateFixedStand(CtrlComponent &ctrlComp,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd);

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


#endif //STATEFIXEDSTAND_H
