//
// Created by tlab-uav on 24-9-18.
//

#include "unitree_guide_controller/FSM/StateTrotting.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/control/CtrlComponent.h>
#include <unitree_guide_controller/control/Estimator.h>
#include <unitree_guide_controller/gait/WaveGenerator.h>

StateTrotting::StateTrotting(CtrlInterfaces &ctrl_interfaces,
                             CtrlComponent &ctrl_component) : FSMState(FSMStateName::TROTTING, "trotting",
                                                                       ctrl_interfaces),
                                                              estimator_(ctrl_component.estimator_),
                                                              robot_model_(ctrl_component.robot_model_),
                                                              balance_ctrl_(ctrl_component.balance_ctrl_),
                                                              wave_generator_(ctrl_component.wave_generator_),
                                                              gait_generator_(ctrl_component) {
    gait_height_ = 0.08;
    Kpp = Vec3(70, 70, 70).asDiagonal();
    Kdp = Vec3(10, 10, 10).asDiagonal();
    kp_w_ = 780;
    Kd_w_ = Vec3(70, 70, 70).asDiagonal();
    Kp_swing_ = Vec3(400, 400, 400).asDiagonal();
    Kd_swing_ = Vec3(10, 10, 10).asDiagonal();

    v_x_limit_ << -0.4, 0.4;
    v_y_limit_ << -0.3, 0.3;
    w_yaw_limit_ << -0.5, 0.5;
    dt_ = 1.0 / ctrl_interfaces_.frequency_;
}

void StateTrotting::enter() {
    pcd_ = estimator_->getPosition();
    pcd_(2) = -estimator_->getFeetPos2Body()(2, 0);
    v_cmd_body_.setZero();
    yaw_cmd_ = estimator_->getYaw();
    Rd = rotz(yaw_cmd_);
    w_cmd_global_.setZero();

    ctrl_interfaces_.control_inputs_.command = 0;
    gait_generator_.restart();
}

void StateTrotting::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    pos_body_ = estimator_->getPosition();
    vel_body_ = estimator_->getVelocity();

    B2G_RotMat = estimator_->getRotation();
    G2B_RotMat = B2G_RotMat.transpose();

    getUserCmd();
    calcCmd();

    gait_generator_.setGait(vel_target_.segment(0, 2), w_cmd_global_(2), gait_height_);
    gait_generator_.generate(pos_feet_global_goal_, vel_feet_global_goal_);

    calcTau();
    calcQQd();

    if (checkStepOrNot()) {
        wave_generator_->status_ = WaveStatus::WAVE_ALL;
    } else {
        wave_generator_->status_ = WaveStatus::STANCE_ALL;
    }

    calcGain();
}

void StateTrotting::exit() {
    wave_generator_->status_ = WaveStatus::SWING_ALL;
}

FSMStateName StateTrotting::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void StateTrotting::getUserCmd() {
    /* Movement */
    v_cmd_body_(0) = invNormalize(ctrl_interfaces_.control_inputs_.ly, v_x_limit_(0), v_x_limit_(1));
    v_cmd_body_(1) = -invNormalize(ctrl_interfaces_.control_inputs_.lx, v_y_limit_(0), v_y_limit_(1));
    v_cmd_body_(2) = 0;

    /* Turning */
    d_yaw_cmd_ = -invNormalize(ctrl_interfaces_.control_inputs_.rx, w_yaw_limit_(0), w_yaw_limit_(1));
    d_yaw_cmd_ = 0.9 * d_yaw_cmd_past_ + (1 - 0.9) * d_yaw_cmd_;
    d_yaw_cmd_past_ = d_yaw_cmd_;
}

void StateTrotting::calcCmd() {
    /* Movement */
    vel_target_ = B2G_RotMat * v_cmd_body_;

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
    yaw_cmd_ = yaw_cmd_ + d_yaw_cmd_ * dt_;
    Rd = rotz(yaw_cmd_);
    w_cmd_global_(2) = d_yaw_cmd_;
}

void StateTrotting::calcTau() {
    pos_error_ = pcd_ - pos_body_;
    vel_error_ = vel_target_ - vel_body_;

    Vec3 dd_pcd = Kpp * pos_error_ + Kdp * vel_error_;
    Vec3 d_wbd = kp_w_ * rotMatToExp(Rd * G2B_RotMat) +
                 Kd_w_ * (w_cmd_global_ - estimator_->getGyroGlobal());

    dd_pcd(0) = saturation(dd_pcd(0), Vec2(-3, 3));
    dd_pcd(1) = saturation(dd_pcd(1), Vec2(-3, 3));
    dd_pcd(2) = saturation(dd_pcd(2), Vec2(-5, 5));

    d_wbd(0) = saturation(d_wbd(0), Vec2(-40, 40));
    d_wbd(1) = saturation(d_wbd(1), Vec2(-40, 40));
    d_wbd(2) = saturation(d_wbd(2), Vec2(-10, 10));

    const Vec34 pos_feet_body_global = estimator_->getFeetPos2Body();
    Vec34 force_feet_global =
            -balance_ctrl_->calF(dd_pcd, d_wbd, B2G_RotMat, pos_feet_body_global, wave_generator_->contact_);


    Vec34 pos_feet_global = estimator_->getFeetPos();
    Vec34 vel_feet_global = estimator_->getFeetVel();

    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            force_feet_global.col(i) =
                    Kp_swing_ * (pos_feet_global_goal_.col(i) - pos_feet_global.col(i)) +
                    Kd_swing_ * (vel_feet_global_goal_.col(i) - vel_feet_global.col(i));
        }
    }

    Vec34 force_feet_body_ = G2B_RotMat * force_feet_global;

    std::vector<KDL::JntArray> current_joints = robot_model_->current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_->getTorque(force_feet_body_.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_interfaces_.joint_torque_command_interface_[i * 3 + j].get().set_value(torque(j));
        }
    }
}

void StateTrotting::calcQQd() {
    const std::vector<KDL::Frame> pos_feet_body = robot_model_->getFeet2BPositions();

    Vec34 pos_feet_target, vel_feet_target;
    for (int i(0); i < 4; ++i) {
        pos_feet_target.col(i) = G2B_RotMat * (pos_feet_global_goal_.col(i) - pos_body_);
        vel_feet_target.col(i) = G2B_RotMat * (vel_feet_global_goal_.col(i) - vel_body_);
    }

    Vec12 q_goal = robot_model_->getQ(pos_feet_target);
    Vec12 qd_goal = robot_model_->getQd(pos_feet_body, vel_feet_target);
    for (int i = 0; i < 12; i++) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(q_goal(i));
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(qd_goal(i));
    }
}

void StateTrotting::calcGain() const {
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_(i) == 0) {
            // swing gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(3);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(2);
            }
        } else {
            // stable gain
            for (int j = 0; j < 3; j++) {
                ctrl_interfaces_.joint_kp_command_interface_[i * 3 + j].get().set_value(0.8);
                ctrl_interfaces_.joint_kd_command_interface_[i * 3 + j].get().set_value(0.8);
            }
        }
    }
}

bool StateTrotting::checkStepOrNot() {
    if (fabs(v_cmd_body_(0)) > 0.03 || fabs(v_cmd_body_(1)) > 0.03 ||
        fabs(pos_error_(0)) > 0.08 || fabs(pos_error_(1)) > 0.08 ||
        fabs(vel_error_(0)) > 0.05 || fabs(vel_error_(1)) > 0.05 ||
        fabs(d_yaw_cmd_) > 0.20) {
        return true;
    }
    return false;
}
