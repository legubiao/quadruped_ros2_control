//
// Created by biao on 24-9-12.
//


#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser/kdl_parser.hpp>

class Robotleg {
public:
    explicit Robotleg(const KDL::Chain &chain);

    ~Robotleg() = default;

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

protected:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pose_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pose_solver_;
};


#endif //ROBOTLEG_H
