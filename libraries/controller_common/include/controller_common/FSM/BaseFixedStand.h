//
// Created by biao on 24-9-10.
//
#pragma once

#include "FSMState.h"

class BaseFixedStand : public FSMState
{
public:
    BaseFixedStand(CtrlInterfaces& ctrl_interfaces,
                   const std::vector<double>& target_pos,
                   double kp,
                   double kd);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

protected:
    double target_pos_[12] = {};
    double start_pos_[12] = {};
    rclcpp::Time start_time_;

    double kp_, kd_;

    double duration_ = 600; // steps
    double percent_ = 0; //%
    double phase = 0.0;
};
