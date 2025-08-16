//
// Created by tlab-uav on 24-9-13.
//

#include "basic_quadruped_controller/FSM/StateFreeStand.h"

#include <basic_quadruped_controller/BasicQuadrupedController.h>

#include "basic_quadruped_controller/common/mathTools.h"
#include "basic_quadruped_controller/common/mathTypes.h"

StateFreeStand::StateFreeStand(CtrlInterfaces &ctrl_interfaces,
                               CtrlComponent &ctrl_component,
                               const double kp,
                               const double kd)
    : FSMState(
          FSMStateName::FREESTAND, "free stand",
          ctrl_interfaces),
      robot_model_(ctrl_component.robot_model_),
      kp_(kp), kd_(kd) {
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
        std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }

    init_joint_pos_ = robot_model_->current_joint_pos_;
    init_foot_pos_ = robot_model_->getFeet2BPositions();


    fl_init_pos_ = init_foot_pos_[0];  // FL (前左) 作为参考足端
    for (auto &foot_pos: init_foot_pos_) {
        foot_pos.translation() -= fl_init_pos_.translation();
        foot_pos.rotation() = Eigen::Matrix3d::Identity();
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StateFreeStand::run(const rclcpp::Time &/*time*/, const rclcpp::Duration &/*period*/) {
    calc_body_target(invNormalize(ctrl_interfaces_.control_inputs_.lx, row_min_, row_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.ly, pitch_min_, pitch_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.rx, yaw_min_, yaw_max_),
                     invNormalize(ctrl_interfaces_.control_inputs_.ry, height_min_, height_max_));
}

void StateFreeStand::exit() {
}

FSMStateName StateFreeStand::checkChange() {
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::FREESTAND;
    }
}

void StateFreeStand::calc_body_target(const float row, const float pitch,
                                      const float yaw, const float height) {
    // 创建目标身体姿态
    pinocchio::SE3 fl_2_body_pos;
    fl_2_body_pos.translation() = -fl_init_pos_.translation();  // 身体相对于FL足端的位置
    fl_2_body_pos.translation().z() += height;
    
    // 设置旋转矩阵（RPY角度）
    Eigen::AngleAxisd roll_angle(row, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(-yaw, Eigen::Vector3d::UnitZ());
    fl_2_body_pos.rotation() = roll_angle * pitch_angle * yaw_angle;

    // 计算每个足端的目标位置
    const pinocchio::SE3 body_2_fl_pos = fl_2_body_pos.inverse();
    std::vector<pinocchio::SE3> goal_pos(4);
    for (int i = 0; i < 4; i++) {
        goal_pos[i] = body_2_fl_pos * init_foot_pos_[i];
    }
    
    // 通过逆运动学计算关节角度
    target_joint_pos_ = robot_model_->getQ(goal_pos);

    // 设置关节位置命令
    for (int i = 0; i < 12; i++) {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            target_joint_pos_(i));
    }
}
