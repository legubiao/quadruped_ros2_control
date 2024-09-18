//
// Created by biao on 24-9-12.
//


#ifndef STATESWINGTEST_H
#define STATESWINGTEST_H
#include "FSMState.h"


class StateSwingTest final : public FSMState {
public:
    explicit StateSwingTest(CtrlComponent &ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:

    void positionCtrl();

    void torqueCtrl() const;

    float _xMin, _xMax;
    float _yMin, _yMax;
    float _zMin, _zMax;

    KDL::Vector Kp, Kd;

    std::vector<KDL::JntArray> init_joint_pos_;
    std::vector<KDL::JntArray> target_joint_pos_;

    std::vector<KDL::Frame> init_foot_pos_;
    std::vector<KDL::Frame> target_foot_pos_;

    KDL::Frame fr_init_pos_;
    KDL::Frame fr_goal_pos_;
};


#endif //STATESWINGTEST_H
