//
// Created by biao on 24-9-12.
//

#include <iostream>
#include <unitree_guide_controller/FSM/StateSwingTest.h>
#include <unitree_guide_controller/common/mathTools.h>

StateSwingTest::StateSwingTest(CtrlComponent ctrlComp): FSMState(
    FSMStateName::SWINGTEST, "swing test", std::move(ctrlComp)) {
    _xMin = -0.15;
    _xMax = 0.10;
    _yMin = -0.15;
    _yMax = 0.15;
    _zMin = -0.05;
    _zMax = 0.20;
}

void StateSwingTest::enter() {
    for (int i = 0; i < 3; i++) {
        ctrlComp_.joint_kp_command_interface_[i].get().set_value(3);
        ctrlComp_.joint_kd_command_interface_[i].get().set_value(2);
    }
    for (int i = 3; i < 12; i++) {
        ctrlComp_.joint_kp_command_interface_[i].get().set_value(180);
        ctrlComp_.joint_kd_command_interface_[i].get().set_value(5);
    }

    Kp = KDL::Vector(20, 20, 50);
    Kd = KDL::Vector(5, 5, 20);

    ctrlComp_.robot_model_.get().update(ctrlComp_);
    init_joint_pos_ = ctrlComp_.robot_model_.get().current_joint_pos_;

    init_foot_pos_ = ctrlComp_.robot_model_.get().getFeet2BPositions(init_joint_pos_);
    target_foot_pos_ = init_foot_pos_;
    fr_init_pos_ = init_foot_pos_[0];
    fr_goal_pos_ = fr_init_pos_;
}

void StateSwingTest::run() {
    if (ctrlComp_.control_inputs_.get().ly > 0) {
        fr_goal_pos_.p.x(invNormalize(ctrlComp_.control_inputs_.get().ly, fr_init_pos_.p.x(),
                                      fr_init_pos_.p.x() + _xMax, 0, 1));
    } else {
        fr_goal_pos_.p.x(invNormalize(ctrlComp_.control_inputs_.get().ly, fr_init_pos_.p.x() + _xMin,
                                      fr_init_pos_.p.x(), -1, 0));
    }
    if (ctrlComp_.control_inputs_.get().lx > 0) {
        fr_goal_pos_.p.y(invNormalize(ctrlComp_.control_inputs_.get().lx, fr_init_pos_.p.y(),
                                      fr_init_pos_.p.y() + _yMax, 0, 1));
    } else {
        fr_goal_pos_.p.y(invNormalize(ctrlComp_.control_inputs_.get().lx, fr_init_pos_.p.y() + _yMin,
                                      fr_init_pos_.p.y(), -1, 0));
    }
    if (ctrlComp_.control_inputs_.get().ry > 0) {
        fr_goal_pos_.p.z(invNormalize(ctrlComp_.control_inputs_.get().ry, fr_init_pos_.p.z(),
                                      fr_init_pos_.p.z() + _zMax, 0, 1));
    } else {
        fr_goal_pos_.p.z(invNormalize(ctrlComp_.control_inputs_.get().ry, fr_init_pos_.p.z() + _zMin,
                                      fr_init_pos_.p.z(), -1, 0));
    }

    ctrlComp_.robot_model_.get().update(ctrlComp_);
    positionCtrl();
    torqueCtrl();
}

void StateSwingTest::exit() {
}

FSMStateName StateSwingTest::checkChange() {
    switch (ctrlComp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::SWINGTEST;
    }
}

void StateSwingTest::positionCtrl() {
    target_foot_pos_[0] = fr_goal_pos_;
    target_joint_pos_ = ctrlComp_.robot_model_.get().getQ(target_foot_pos_);
    for (int i = 0; i < 4; i++) {
        ctrlComp_.joint_position_command_interface_[i * 3].get().set_value(target_joint_pos_[i](0));
        ctrlComp_.joint_position_command_interface_[i * 3 + 1].get().set_value(target_joint_pos_[i](1));
        ctrlComp_.joint_position_command_interface_[i * 3 + 2].get().set_value(target_joint_pos_[i](2));
    }
}

void StateSwingTest::torqueCtrl() const {
    const KDL::Frame fr_current_pos = ctrlComp_.robot_model_.get().getFeet2BPositions(0);
    KDL::Jacobian fr_jaco = ctrlComp_.robot_model_.get().getJacobian(0);

    const KDL::Vector pos_goal = fr_goal_pos_.p;
    const KDL::Vector pos0 = fr_current_pos.p;
    const Eigen::Matrix<double, 3, Eigen::Dynamic> linear_jacobian = fr_jaco.data.topRows(3);
    Eigen::Product<Eigen::Matrix<double, 3, -1>, Eigen::Matrix<double, -1, 1> > vel_eigen =
            linear_jacobian * ctrlComp_.robot_model_.get().current_joint_vel_[0].data;
    const KDL::Vector vel0(vel_eigen(0), vel_eigen(1), vel_eigen(2));

    const KDL::Vector force0 = Kp * (pos_goal - pos0) + Kd * (-vel0);
    const KDL::Wrench wrench0(force0, KDL::Vector::Zero()); // 假设力矩为零
    const KDL::Wrenches wrenches(1, wrench0); // 创建一个包含 wrench0 的容器
    KDL::JntArray torque0 = ctrlComp_.robot_model_.get().getTorque(wrenches, 0);

    for (int i = 0; i < 3; i++) {
        ctrlComp_.joint_effort_command_interface_[i].get().set_value(torque0(i));
    }
}
