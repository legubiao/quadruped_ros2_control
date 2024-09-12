//
// Created by biao on 24-9-12.
//

#include <iostream>
#include <unitree_guide_controller/robotics/RobotLeg.h>

Robotleg::Robotleg(const KDL::Chain &chain) {
    chain_ = chain;

    fk_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    ik_pose_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain);
}

KDL::Frame Robotleg::calcPEe2B(const KDL::JntArray &joint_positions) const {
    KDL::Frame pEe;
    if (fk_pose_solver_->JntToCart(joint_positions, pEe) < 0) {
        std::cerr << "Failed to calculate forward kinematics" << std::endl;
    }
    return pEe;
}

KDL::JntArray Robotleg::calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const {
    KDL::JntArray q(chain_.getNrOfJoints());
    if (ik_pose_solver_->CartToJnt(q_init, pEe, q) < 0) {
        std::cerr << "Failed to calculate inverse kinematics" << std::endl;
    }
    return q;
}
