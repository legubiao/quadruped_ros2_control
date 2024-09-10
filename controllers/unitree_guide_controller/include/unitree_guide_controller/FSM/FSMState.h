//
// Created by tlab-uav on 24-9-6.
//

#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <utility>
#include <unitree_guide_controller/common/enumClass.h>
#include <unitree_guide_controller/common/interface.h>

class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName &stateName, std::string stateNameString, CtrlComponent ctrlComp)
        : state_name(stateName),
          state_name_string(std::move(stateNameString)),
          ctrlComp_(std::move(ctrlComp)) {
    }

    virtual void enter() = 0;

    virtual void run() = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    CtrlComponent ctrlComp_;
};

#endif //FSMSTATE_H
