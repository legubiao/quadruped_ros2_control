//
// Created by biao on 24-9-10.
//

#include "FSMState.h"

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H



class StateFixedStand : public FSMState {
    explicit StateFixedStand(CtrlComponent ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;
};



#endif //STATEFIXEDSTAND_H
