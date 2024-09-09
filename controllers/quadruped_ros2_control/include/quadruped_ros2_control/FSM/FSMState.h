//
// Created by tlab-uav on 24-9-6.
//

#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include "quadruped_ros2_control/common/enumClass.h"

class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(FSMStateName stateName, std::string stateNameString);

    virtual void enter();

    virtual void run();

    virtual void exit();

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;
};


#endif //FSMSTATE_H
