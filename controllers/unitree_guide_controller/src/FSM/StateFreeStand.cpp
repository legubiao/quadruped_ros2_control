//
// Created by tlab-uav on 24-9-13.
//

#include "unitree_guide_controller/FSM/StateFreeStand.h"
#include "unitree_guide_controller/common/mathTools.h"

StateFreeStand::StateFreeStand(CtrlComponent ctrl_component) : FSMState(FSMStateName::FREESTAND, "free stand",
                                                                        std::move(ctrl_component)) {
    row_max_ = 20 * M_PI / 180;
    row_min_ = -row_max_;
    pitch_max_ = 15 * M_PI / 180;
    pitch_min_ = -pitch_max_;
    yaw_max_ = 20 * M_PI / 180;
    yaw_min_ = -yaw_max_;
    height_max_ = 0.1;
    height_min_ = -height_max_;
}

void StateFreeStand::enter() {
    for (int i = 0; i < 12; i++) {
        ctrlComp_.joint_kp_command_interface_[i].get().set_value(180);
        ctrlComp_.joint_kd_command_interface_[i].get().set_value(5);
    }
    ctrlComp_.robot_model_.get().update(ctrlComp_);

    init_joint_pos_ = ctrlComp_.robot_model_.get().current_joint_pos_;
    init_foot_pos_ = ctrlComp_.robot_model_.get().getFeet2BPositions(init_joint_pos_);


    fr_init_pos_ = init_foot_pos_[0];
    for (auto &foot_pos: init_foot_pos_) {
        foot_pos.p -= fr_init_pos_.p;
        foot_pos.M = KDL::Rotation::RPY(0, 0, 0);
    }
}

void StateFreeStand::run() {
    ctrlComp_.robot_model_.get().update(ctrlComp_);
    calc_body_target(invNormalize(ctrlComp_.control_inputs_.get().lx, row_min_, row_max_),
                     invNormalize(ctrlComp_.control_inputs_.get().ly, pitch_min_, pitch_max_),
                     invNormalize(ctrlComp_.control_inputs_.get().rx, yaw_min_, yaw_max_),
                     invNormalize(ctrlComp_.control_inputs_.get().ry, height_min_, height_max_));
}

void StateFreeStand::exit() {
}

FSMStateName StateFreeStand::checkChange() {
    switch (ctrlComp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::FREESTAND;
    }
}

void StateFreeStand::calc_body_target(const float row, const float pitch,
                                      const float yaw, const float height) {
    KDL::Frame fr_2_body_pos;
    fr_2_body_pos.p = -fr_init_pos_.p;
    fr_2_body_pos.p.z(fr_2_body_pos.p.z() + height);
    fr_2_body_pos.M = KDL::Rotation::RPY(row, pitch, yaw);

    const KDL::Frame body_2_fr_pos = fr_2_body_pos.Inverse();
    std::vector goal_pos(4, KDL::Frame::Identity());
    for (int i = 0; i < 4; i++) {
        goal_pos[i] = body_2_fr_pos * init_foot_pos_[i];
    }
    target_joint_pos_ = ctrlComp_.robot_model_.get().getQ(goal_pos);

    for (int i = 0; i < 4; i++) {
        ctrlComp_.joint_position_command_interface_[i * 3].get().set_value(target_joint_pos_[i](0));
        ctrlComp_.joint_position_command_interface_[i * 3 + 1].get().set_value(target_joint_pos_[i](1));
        ctrlComp_.joint_position_command_interface_[i * 3 + 2].get().set_value(target_joint_pos_[i](2));
    }
}
