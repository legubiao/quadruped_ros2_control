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
    _gaitHeight = 0.08;
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
    _pcd = estimator_.getPosition();
    _vCmdBody.setZero();
    _yawCmd = estimator_.getYaw();
    Rd = Mat3(KDL::Rotation::RPY(0, 0, _yawCmd).Inverse().data);
    _wCmdGlobal.setZero();

    ctrl_comp_.control_inputs_.get().command = 0;
    gait_generator_.restart();
}

void StateTrotting::run() {
    _posBody = estimator_.getPosition();
    _velBody = estimator_.getVelocity();
    _posFeet2BGlobal = estimator_.getFeetPos2Body();
    _posFeetGlobal = estimator_.getFeetPos();
    B2G_RotMat = Eigen::Matrix3d(estimator_.getRotation().data);
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    calcCmd();

    gait_generator_.setGait(_vCmdGlobal.segment(0, 2), _wCmdGlobal(2), _gaitHeight);
    gait_generator_.run(_posFeetGlobalGoal, _velFeetGlobalGoal);

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
    _vCmdGlobal = B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) =
            saturation(_vCmdGlobal(0), Vec2(_velBody(0) - 0.2, _velBody(0) + 0.2));
    _vCmdGlobal(1) =
            saturation(_vCmdGlobal(1), Vec2(_velBody(1) - 0.2, _velBody(1) + 0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * dt_,
                         Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * dt_,
                         Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * dt_;
    Rd = Mat3(KDL::Rotation::RPY(0, 0, _yawCmd).Inverse().data);
    _wCmdGlobal(2) = _dYawCmd;
}

void StateTrotting::calcTau() {
    pos_error_ = _pcd - _posBody;
    vel_error_ = _vCmdGlobal - _velBody;

    _ddPcd = Kpp * pos_error_ + Kdp * vel_error_;
    _dWbd = _kpw * rotMatToExp(Rd * G2B_RotMat) +
            Kdw * (_wCmdGlobal - estimator_.getGlobalGyro());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    force_feet_global_ =
            -balance_ctrl_.calF(_ddPcd, _dWbd, B2G_RotMat, _posFeet2BGlobal, wave_generator_.contact_);

    for (int i(0); i < 4; ++i) {
        if (wave_generator_.contact_(i) == 0) {
            force_feet_global_.col(i) =
                    KpSwing * (_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) +
                    KdSwing * (_velFeetGlobalGoal.col(i) - _velFeetGlobal.col(i));
        }
    }

    force_feet_body_ = G2B_RotMat * force_feet_global_;

    std::vector<KDL::JntArray> current_joints = robot_model_.current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_.getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_comp_.joint_effort_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> _posFeet2B = robot_model_.getFeet2BPositions();

    for (int i(0); i < 4; ++i) {
        _posFeet2BGoal.col(i) = G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) -
        // _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i))
        // );  //  c.f formula (6.12)
    }

    Vec12 q_goal = robot_model_.getQ(_posFeet2BGoal);
    Vec12 qd_goal = robot_model_.getQd(_posFeet2B, _velFeet2BGoal);
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
