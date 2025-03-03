//
// Created by biao on 24-9-12.
//


#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <controller_common/common/mathTypes.h>

class RobotLeg {
public:
    explicit RobotLeg(const KDL::Chain &chain);

    ~RobotLeg() = default;

    /**
     * Use forward kinematic to calculate the Pose of End effector to Body frame.
     * @param joint_positions Leg joint positions
     * @return Pose of End effector to Body frame
     */
    [[nodiscard]] KDL::Frame calcPEe2B(const KDL::JntArray &joint_positions) const;

    /**
     * Use inverse kinematic to calculate the joint positions.
     * @param pEe target position of end effector
     * @param q_init current joint positions
     * @return target joint positions
     */
    [[nodiscard]] KDL::JntArray calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const;

    /**
     * Calculate the current jacobian matrix.
     * @param joint_positions Leg joint positions
     * @return jacobian matrix
     */
    [[nodiscard]] KDL::Jacobian calcJaco(const KDL::JntArray &joint_positions) const;

    /**
     * Calculate the torque based on joint positions and end force
     * @param joint_positions current joint positions
     * @param force foot end force
     * @return joint torque
     */
    [[nodiscard]] KDL::JntArray calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const;

protected:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pose_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pose_solver_;
};


#endif //ROBOTLEG_H
