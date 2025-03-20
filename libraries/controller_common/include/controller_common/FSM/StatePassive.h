//
// Created by tlab-uav on 24-9-6.
//

#ifndef STATEPASSIVE_H
#define STATEPASSIVE_H
#include "FSMState.h"

class StatePassive final : public FSMState
{
public:
    explicit StatePassive(CtrlInterfaces& ctrl_interfaces);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEPASSIVE_H
