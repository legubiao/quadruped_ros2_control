//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include "FSMState.h"

class StateFixedStand : public FSMState {
public:
    explicit StateFixedStand(CtrlComponent ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;
};


#endif //STATEFIXEDSTAND_H
