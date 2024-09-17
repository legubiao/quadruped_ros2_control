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
    pcd_ = ctrl_comp_.estimator_.get().getPosition();
    pcdInit_ = pcd_;
    RdInit_ = ctrl_comp_.estimator_.get().getRotation();
}

void StateBalanceTest::run() {
    pcd_(0) = pcdInit_(0) + invNormalize(ctrl_comp_.control_inputs_.get().ly, _xMin, _xMax);
    pcd_(1) = pcdInit_(1) - invNormalize(ctrl_comp_.control_inputs_.get().lx, _yMin, _yMax);
    pcd_(2) = pcdInit_(2) + invNormalize(ctrl_comp_.control_inputs_.get().ry, _zMin, _zMax);
    const float yaw = invNormalize(ctrl_comp_.control_inputs_.get().rx, _yawMin, _yawMax);
    Rd_ = KDL::Rotation::RPY(0, 0, yaw) * RdInit_;

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
    // expected body acceleration
    _ddPcd = Kp_p_ * Vec3((pcd_ - pose_body_).data) + Kd_p_ * Vec3(
                 (KDL::Vector(0, 0, 0) - vel_body_).data);

    // expected body angular acceleration
    const KDL::Rotation B2G_Rotation = ctrl_comp_.estimator_.get().getRotation();
    const KDL::Rotation G2B_Rotation = B2G_Rotation.Inverse();
    _dWbd = kp_w_ * rotationToExp(Rd_ * G2B_Rotation) +
            Kd_w_ * (Vec3(0, 0, 0) - Vec3(ctrl_comp_.estimator_.get().getGlobalGyro().data));

    // calculate foot force
    const std::vector contact(4, 1);
    const std::vector<KDL::Vector> foot_force = ctrl_comp_.balance_ctrl_.get().calF(_ddPcd, _dWbd, B2G_Rotation,
        ctrl_comp_.estimator_.get().
        getFootPos2Body(), contact);

    std::vector<KDL::JntArray> current_joints = ctrl_comp_.robot_model_.get().current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        std::cout<<Vec3(foot_force[i].data).transpose()<<std::endl;
        KDL::JntArray torque = ctrl_comp_.robot_model_.get().getTorque(B2G_Rotation*(-foot_force[i]), i);
        for (int j = 0; j < 3; j++) {
            ctrl_comp_.joint_effort_command_interface_[i * 3 + j].get().set_value(torque(j));
            ctrl_comp_.joint_position_command_interface_[i * 3 + j].get().set_value(current_joints[i](j));
        }
    }
}
