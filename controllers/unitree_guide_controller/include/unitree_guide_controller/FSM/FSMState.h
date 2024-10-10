//
// Created by tlab-uav on 24-9-6.
//

#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <utility>
#include <unitree_guide_controller/common/enumClass.h>
#include <unitree_guide_controller/control/CtrlComponent.h>

class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(const FSMStateName &state_name, std::string state_name_string, CtrlComponent &ctrl_comp)
        : state_name(state_name),
          state_name_string(std::move(state_name_string)),
          ctrl_comp_(ctrl_comp) {
    }

    virtual void enter() = 0;

    virtual void run() = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    CtrlComponent &ctrl_comp_;
};

#endif //FSMSTATE_H
