//
// Created by tlab-uav on 24-9-16.
//

#ifndef STATEBALANCETEST_H
#define STATEBALANCETEST_H
#include "FSMState.h"


class StateBalanceTest final : public FSMState {
public:
    explicit StateBalanceTest(CtrlComponent &ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void calcTorque();

    Estimator& estimator_;
    QuadrupedRobot& robot_model_;
    BalanceCtrl& balance_ctrl_;

    Vec3 pcd_, pcd_init_;
    RotMat Rd_;
    KDL::Rotation init_rotation_;

    Vec3 pose_body_, vel_body_;

    double kp_w_;
    Mat3 Kp_p_, Kd_p_, Kd_w_;
    Vec3 dd_pcd_, d_wbd_;

    float _xMax, _xMin;
    float _yMax, _yMin;
    float _zMax, _zMin;
    float _yawMax, _yawMin;
};


#endif //STATEBALANCETEST_H
