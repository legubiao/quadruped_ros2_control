//
// Created by biao on 24-9-12.
//

#include <unitree_guide_controller/robotics/QuadrupedRobot.h>

QuadrupedRobot::QuadrupedRobot(const std::string &robot_description) {
    KDL::Tree robot_tree;
    kdl_parser::treeFromString(robot_description, robot_tree);

    robot_tree.getChain("base", "FR_foot", fr_chain_);
    robot_tree.getChain("base", "FL_foot", fl_chain_);
    robot_tree.getChain("base", "RR_foot", rr_chain_);
    robot_tree.getChain("base", "RL_foot", rl_chain_);

    robot_legs_.emplace_back(fr_chain_);
    robot_legs_.emplace_back(fl_chain_);
    robot_legs_.emplace_back(rr_chain_);
    robot_legs_.emplace_back(rl_chain_);

    // calculate total mass from urdf
    double totoal_mass = 0.0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        totoal_mass += snd.segment.getInertia().getMass();
    }
    mass_ = totoal_mass;
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list,
                                                const std::vector<KDL::JntArray> &q_init) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result.push_back(robot_legs_[i].calcQ(pEe_list[i], q_init[i]));
    }
    return result;
}

std::vector<KDL::Frame> QuadrupedRobot::getFeet2BPositions(const std::vector<KDL::JntArray> &joint_positions) const {
    std::vector<KDL::Frame> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result.push_back(robot_legs_[i].calcPEe2B(joint_positions[i]));
    }
    return result;
}
