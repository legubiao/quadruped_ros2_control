//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <unitree_guide_controller/common/mathTools.h>

StateTrotting::StateTrotting(CtrlComponent &ctrlComp) : FSMState(FSMStateName::TROTTING, "trotting", ctrlComp),
                                                        estimator_(ctrlComp.estimator_),
                                                        robot_model_(ctrlComp.robot_model_),
                                                        balance_ctrl_(ctrlComp.balance_ctrl_),
                                                        wave_generator_(ctrl_comp_.wave_generator_),
                                                        gait_generator_(ctrlComp) {
    gait_height_ = 0.08;
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780;
    Kdw = Vec3(70, 70, 70).asDiagonal();
    KpSwing = Vec3(400, 400, 400).asDiagonal();
    KdSwing = Vec3(10, 10, 10).asDiagonal();

    _vxLim << -0.4, 0.4;
    _vyLim << -0.3, 0.3;
    _wyawLim << -0.5, 0.5;
    dt_ = 1.0 / ctrl_comp_.frequency_;
}

void StateTrotting::enter() {
    pcd_ = estimator_.getPosition();
    _vCmdBody.setZero();
    _yawCmd = estimator_.getYaw();
    Rd = Mat3(KDL::Rotation::RPY(0, 0, _yawCmd).Inverse().data);
    _wCmdGlobal.setZero();

    ctrl_comp_.control_inputs_.get().command = 0;
    gait_generator_.restart();
}

void StateTrotting::run() {
    pos_body_ = estimator_.getPosition();
    vel_body_ = estimator_.getVelocity();

    B2G_RotMat = estimator_.getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    calcCmd();

    gait_generator_.setGait(vel_target_.segment(0, 2), _wCmdGlobal(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    calcTau();
    calcQQd();

    if (checkStepOrNot()) {
        wave_generator_.status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_.status_ = WaveStatus::STANCE_ALL;
    }

    calcGain();
}

void StateTrotting::exit() {
    wave_generator_.status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    switch (ctrl_comp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    /* Movement */
    _vCmdBody(0) = invNormalize(ctrl_comp_.control_inputs_.get().ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(ctrl_comp_.control_inputs_.get().lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(ctrl_comp_.control_inputs_.get().rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void StateTrotting::calcCmd() {
    /* Movement */
    vel_target_ = B2G_RotMat * _vCmdBody;

    vel_target_(0) =
            saturation(vel_target_(0), Vec2(vel_body_(0) - 0.2, vel_body_(0) + 0.2));
    vel_target_(1) =
            saturation(vel_target_(1), Vec2(vel_body_(1) - 0.2, vel_body_(1) + 0.2));

    pcd_(0) = saturation(pcd_(0) + vel_target_(0) * dt_,
                         Vec2(pos_body_(0) - 0.05, pos_body_(0) + 0.05));
    pcd_(1) = saturation(pcd_(1) + vel_target_(1) * dt_,
                         Vec2(pos_body_(1) - 0.05, pos_body_(1) + 0.05));

    vel_target_(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * dt_;
    Rd = Mat3(KDL::Rotation::RPY(0, 0, _yawCmd).Inverse().data);
    _wCmdGlobal(2) = _dYawCmd;
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = _kpw * rotMatToExp(Rd * G2B_RotMat) +
                 Kdw * (_wCmdGlobal - estimator_.getGlobalGyro());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet2_b_global = estimator_.getFeetPos2Body();
    Vec34 force_feet_global =
            -balance_ctrl_.calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet2_b_global, wave_generator_.contact_);


    Vec34 pos_feet_global = estimator_.getFeetPos();
    Vec34 vel_feet_global = estimator_.getFeetVel();
    for (int i(0); i < 4; ++i) {
        if (wave_generator_.contact_(i) == 0) {
            force_feet_global.col(i) =
                    KpSwing * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    KdSwing * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    std::vector<KDL::JntArray> current_joints = robot_model_.current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_.getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_comp_.joint_effort_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet2_b = robot_model_.getFeet2BPositions();

    Vec34 pos_feet2_b_goal, vel_feet2_b_goal;
    for (int i(0); i < 4; ++i) {
        pos_feet2_b_goal.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet2_b_goal.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) -
        // _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i))
        // );  //  c.f formula (6.12)
    }

    Vec12 q_goal = robot_model_.getQ(pos_feet2_b_goal);
    Vec12 qd_goal = robot_model_.getQd(pos_feet2_b, vel_feet2_b_goal);
    for (int i = 0; i < 12; i++) {
        ctrl_comp_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_.contact_(i) == 0) {
            for (int j = 0; j < 3; j++) {
                ctrl_comp_.joint_kp_command_interface_[i * 3 + j].get().set_value(3);
                ctrl_comp_.joint_kd_command_interface_[i * 3 + j].get().set_value(2);
            }
        } else {
            for (int j = 0; j < 3; j++) {
                ctrl_comp_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                ctrl_comp_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}

bool StateTrotting::checkStepOrNot() {
    if (fabs(_vCmdBody(0)) > 0.03 || fabs(_vCmdBody(1)) > 0.03 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.05 || fabs(vel_error_(1)) > 0.05 ||
        fabs(_dYawCmd) > 0.20) {
        return true;
    }
    return false;
}
