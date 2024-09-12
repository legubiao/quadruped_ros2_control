//
// Created by biao on 24-9-12.
//

#include <iostream>
#include <unitree_guide_controller/robotics/QuadrupedRobot.h>

void QuadrupedRobot::init(const std::string &robot_description) {
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);

    robot_tree.getChain("base", "FR_foot", fr_chain_);
    robot_tree.getChain("base", "FL_foot", fl_chain_);
    robot_tree.getChain("base", "RR_foot", rr_chain_);
    robot_tree.getChain("base", "RL_foot", rl_chain_);

    robot_legs_.resize(4);
    robot_legs_[0] = std::make_shared<RobotLeg>(fr_chain_);
    robot_legs_[1] = std::make_shared<RobotLeg>(fl_chain_);
    robot_legs_[2] = std::make_shared<RobotLeg>(rr_chain_);
    robot_legs_[3] = std::make_shared<RobotLeg>(rl_chain_);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // calculate total mass from urdf
    mass_ = 0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        mass_ += snd.segment.getInertia().getMass();
    }
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list,
                                                const std::vector<KDL::JntArray> &q_init) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], q_init[i]);
    }
    return result;
}

std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions(const std::vector<KDL::JntArray> &joint_positions) const {
    std::vector<KDL::Frame> result;
    result.resize(4);
    for (int i = 0; i < 4; i++) {
        result[i] = robot_legs_[i]->calcPEe2B(joint_positions[i]);
    }
    return result;
}

KDL::Frame QuadrupedRobot::getFeet2BPositions(const KDL::JntArray &joint_positions, const int index) const {
    return robot_legs_[index]->calcPEe2B(joint_positions);
}

KDL::Jacobian QuadrupedRobot::getJacobian(const KDL::JntArray &joint_positions, const int index) const {
    return robot_legs_[index]->calcJaco(joint_positions);
}

KDL::JntArray QuadrupedRobot::getTorque(const KDL::JntArray &joint_positions, const KDL::JntArray &joint_velocities,
                                        const KDL::Wrenches &force, const int index) const {
    return robot_legs_[index]->calcTorque(joint_positions, joint_velocities, force);
}
