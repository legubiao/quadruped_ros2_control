//
// Created by tlab-uav on 24-9-18.
//

#ifndef STATETROTTING_H
#define STATETROTTING_H
#include <unitree_guide_controller/gait/GaitGenerator.h>

#include "FSMState.h"


class StateTrotting final : public FSMState {
public:
    explicit StateTrotting(CtrlComponent &ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void getUserCmd();

    void calcCmd();

    void calcTau();

    void calcQQd();

    void calcGain() const;

    /**
     * Check whether the robot should take a step or not
     * @return
     */
    bool checkStepOrNot();

    Estimator &estimator_;
    QuadrupedRobot &robot_model_;
    BalanceCtrl &balance_ctrl_;
    WaveGenerator &wave_generator_;

    GaitGenerator gait_generator_;

    // Robot State
    Vec3 _posBody, _velBody;
    double _yaw{}, _dYaw{};
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat B2G_RotMat, G2B_RotMat;
    Vec12 _q;


    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double dt_;
    double _yawCmd{}, _dYawCmd{};
    double _dYawCmdPast{};
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 force_feet_global_, force_feet_body_;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kdw;
    double _kpw;
    Mat3 KpSwing, KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
};


#endif //STATETROTTING_H
