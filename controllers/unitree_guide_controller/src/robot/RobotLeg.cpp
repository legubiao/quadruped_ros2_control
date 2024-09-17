//
// Created by biao on 24-9-12.
//

#include <memory>
#include "unitree_guide_controller/robot/RobotLeg.h"

#include <unitree_guide_controller/common/mathTypes.h>

RobotLeg::RobotLeg(const KDL::Chain &chain) {
    chain_ = chain;

    fk_pose_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
    ik_pose_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain);
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

KDL::JntArray RobotLeg::calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const {
    const Eigen::Matrix<double, 3, Eigen::Dynamic> jacobian = calcJaco(joint_positions).data.topRows(3);
    Eigen::VectorXd torque_eigen = jacobian.transpose() * force;
    KDL::JntArray torque(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        torque(i) = torque_eigen(i);
    }
    return torque;
}
