//
// Created by biao on 24-9-12.
//

#include <memory>
#include <unitree_guide_controller/robotics/RobotLeg.h>

RobotLeg::RobotLeg(const KDL::Chain &chain) {
    chain_ = chain;

    fk_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    ik_pose_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain);
    id_solver_ = std::make_shared<KDL::ChainIdSolver_RNE>(chain, KDL::Vector(0, 0, -9.81));
}

KDL::Frame RobotLeg::calcPEe2B(const KDL::JntArray &joint_positions) const {
    KDL::Frame pEe;
    fk_pose_solver_->JntToCart(joint_positions, pEe);
    return pEe;
}

KDL::JntArray RobotLeg::calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const {
    KDL::JntArray q(chain_.getNrOfJoints());
    ik_pose_solver_->CartToJnt(q_init, pEe, q);
    return q;
}

KDL::Jacobian RobotLeg::calcJaco(const KDL::JntArray &joint_positions) const {
    KDL::Jacobian jacobian(chain_.getNrOfJoints());
    jac_solver_->JntToJac(joint_positions, jacobian);
    return jacobian;
}

KDL::JntArray RobotLeg::calcTorque(const KDL::JntArray &joint_positions, const KDL::JntArray &joint_velocities,
                                   const KDL::Wrenches &force) const {
    KDL::JntArray torque(chain_.getNrOfJoints());
    id_solver_->CartToJnt(joint_positions, joint_velocities, KDL::JntArray(chain_.getNrOfJoints()), force,
                              torque);
    return torque;
}
