//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/FSM/StateBalanceTest.h"

#include <unitree_guide_controller/common/mathTools.h>

StateBalanceTest::StateBalanceTest(CtrlComponent ctrlComp) : FSMState(FSMStateName::BALANCETEST, "balance test",
                                                                      std::move(ctrlComp)) {
    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    Kp_p_ = Vec3(150, 150, 150).asDiagonal();
    Kd_p_ = Vec3(25, 25, 25).asDiagonal();

    kp_w_ = 200;
    Kd_w_ = Vec3(30, 30, 30).asDiagonal();
}

void StateBalanceTest::enter() {
    pcdInit_ = ctrl_comp_.estimator_.get().getPosition();
    pcd_ = pcdInit_;
    init_rotation_ = ctrl_comp_.estimator_.get().getRotation();
}

void StateBalanceTest::run() {
    pcd_(0) = pcdInit_(0) + invNormalize(ctrl_comp_.control_inputs_.get().ly, _xMin, _xMax);
    pcd_(1) = pcdInit_(1) - invNormalize(ctrl_comp_.control_inputs_.get().lx, _yMin, _yMax);
    pcd_(2) = pcdInit_(2) + invNormalize(ctrl_comp_.control_inputs_.get().ry, _zMin, _zMax);
    const float yaw = invNormalize(ctrl_comp_.control_inputs_.get().rx, _yawMin, _yawMax);
    Rd_ = Mat3((KDL::Rotation::RPY(0, 0, yaw).Inverse() * init_rotation_).data);

    pose_body_ = ctrl_comp_.estimator_.get().getPosition();
    vel_body_ = ctrl_comp_.estimator_.get().getVelocity();

    for (int i = 0; i < 12; i++) {
        ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0.8);
        ctrl_comp_.joint_kd_command_interface_[i].get().set_value(0.8);
    }

    calcTorque();
}

void StateBalanceTest::exit() {
}

FSMStateName StateBalanceTest::checkChange() {
    switch (ctrl_comp_.control_inputs_.get().command) {
        case 1:
            return FSMStateName::FIXEDDOWN;
        case 2:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::BALANCETEST;
    }
}

void StateBalanceTest::calcTorque() {
    const auto B2G_Rotation = Eigen::Matrix3d(ctrl_comp_.estimator_.get().getRotation().data);
    const RotMat G2B_Rotation = B2G_Rotation.transpose();

    // expected body acceleration
    dd_pcd_ = Kp_p_ * Vec3((pcd_ - pose_body_).data) + Kd_p_ * Vec3((-vel_body_).data);

    // expected body angular acceleration
    d_wbd_ = kp_w_ * rotMatToExp(Rd_ * G2B_Rotation) +
             Kd_w_ * (Vec3(0, 0, 0) - Vec3((-ctrl_comp_.estimator_.get().getGlobalGyro()).data));

    // calculate foot force
    const std::vector contact(4, 1);
    const Vec34 foot_force = G2B_Rotation * ctrl_comp_.balance_ctrl_.get().calF(dd_pcd_, -d_wbd_, B2G_Rotation,
                                 ctrl_comp_.estimator_.get().
                                 getFootPos2Body(), contact);

    std::vector<KDL::JntArray> current_joints = ctrl_comp_.robot_model_.get().current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = ctrl_comp_.robot_model_.get().getTorque(-foot_force.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_comp_.joint_effort_command_interface_[i * 3 + j].get().set_value(torque(j));
            ctrl_comp_.joint_position_command_interface_[i * 3 + j].get().set_value(current_joints[i](j));
        }
    }
}
