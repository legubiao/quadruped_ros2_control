//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/FSM/StateBalanceTest.h"

#include "unitree_guide_controller/common/mathTools.h"

StateBalanceTest::StateBalanceTest(CtrlComponent &ctrlComp) : FSMState(FSMStateName::BALANCETEST, "balance test",
                                                                       ctrlComp),
                                                              estimator_(ctrlComp.estimator_),
                                                              robot_model_(ctrlComp.robot_model_),
                                                              balance_ctrl_(ctrlComp.balance_ctrl_),
                                                              wave_generator_(ctrl_comp_.wave_generator_) {
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
    pcd_init_ = estimator_.getPosition();
    pcd_ = pcd_init_;
    init_rotation_ = estimator_.getRotation();
    wave_generator_.status_ = WaveStatus::STANCE_ALL;
}

void StateBalanceTest::run() {
    pcd_(0) = pcd_init_(0) + invNormalize(ctrl_comp_.control_inputs_.get().ly, _xMin, _xMax);
    pcd_(1) = pcd_init_(1) - invNormalize(ctrl_comp_.control_inputs_.get().lx, _yMin, _yMax);
    pcd_(2) = pcd_init_(2) + invNormalize(ctrl_comp_.control_inputs_.get().ry, _zMin, _zMax);
    const float yaw = invNormalize(ctrl_comp_.control_inputs_.get().rx, _yawMin, _yawMax);
    Rd_ = Mat3((KDL::Rotation::RPY(0, 0, yaw).Inverse() * init_rotation_).data);

    pose_body_ = estimator_.getPosition();
    vel_body_ = estimator_.getVelocity();

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
    const auto B2G_Rotation = Eigen::Matrix3d(estimator_.getRotation().data);
    const RotMat G2B_Rotation = B2G_Rotation.transpose();

    // expected body acceleration
    dd_pcd_ = Kp_p_ * (pcd_ - pose_body_) + Kd_p_ * -vel_body_;

    // expected body angular acceleration
    d_wbd_ = -kp_w_ * rotMatToExp(Rd_ * G2B_Rotation) +
             Kd_w_ * -estimator_.getGlobalGyro();

    // calculate foot force
    const Vec34 foot_force = G2B_Rotation * balance_ctrl_.calF(dd_pcd_, d_wbd_, B2G_Rotation,
                                                               estimator_.getFeetPos2Body(), wave_generator_.contact_);

    std::vector<KDL::JntArray> current_joints = robot_model_.current_joint_pos_;
    for (int i = 0; i < 4; i++) {
        KDL::JntArray torque = robot_model_.getTorque(-foot_force.col(i), i);
        for (int j = 0; j < 3; j++) {
            ctrl_comp_.joint_effort_command_interface_[i * 3 + j].get().set_value(torque(j));
            ctrl_comp_.joint_position_command_interface_[i * 3 + j].get().set_value(current_joints[i](j));
        }
    }
}
