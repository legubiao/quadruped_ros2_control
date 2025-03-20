//
// Created by tlab-uav on 24-9-6.
//

#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <utility>
#include <controller_common/common/enumClass.h>
#include <controller_common/CtrlInterfaces.h>
#include <rclcpp/time.hpp>

class FSMState
{
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName& state_name, std::string state_name_string, CtrlInterfaces& ctrl_interfaces)
        : state_name(state_name),
          state_name_string(std::move(state_name_string)),
          ctrl_interfaces_(ctrl_interfaces)
    {
    }

    virtual void enter() = 0;

    virtual void run(const rclcpp::Time& time,
                     const rclcpp::Duration& period) = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    CtrlInterfaces& ctrl_interfaces_;
};

#endif //FSMSTATE_H
