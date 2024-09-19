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
    Vec3 pos_body_, vel_body_;
    RotMat B2G_RotMat, G2B_RotMat;


    // Robot command
    Vec3 pcd_;
    Vec3 vel_target_, _vCmdBody;
    double dt_;
    double _yawCmd{}, _dYawCmd{};
    double _dYawCmdPast{};
    Vec3 _wCmdGlobal;
    Vec34 pos_feet_global_goal_, vel_feet_global_goal_;
    RotMat Rd;

    // Control Parameters
    double gait_height_;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kd_w_;
    double kp_w_;
    Mat3 Kp_swing_, Kd_swing_;
    Vec2 _vxLim, _vyLim, _wyawLim;
};


#endif //STATETROTTING_H
