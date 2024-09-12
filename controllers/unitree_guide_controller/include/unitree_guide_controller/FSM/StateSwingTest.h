//
// Created by biao on 24-9-12.
//


#ifndef STATESWINGTEST_H
#define STATESWINGTEST_H
#include "FSMState.h"


class StateSwingTest final : public FSMState {
public:
    explicit StateSwingTest(CtrlComponent ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    float _xMin, _xMax;
    float _yMin, _yMax;
    float _zMin, _zMax;
};


#endif //STATESWINGTEST_H
