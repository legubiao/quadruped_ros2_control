//
// Created by biao on 24-9-12.
//

#include <iostream>
#include "unitree_guide_controller/control/CtrlComponent.h"
#include "unitree_guide_controller/robot/QuadrupedRobot.h"

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

    current_joint_pos_.resize(4);
    current_joint_vel_.resize(4);

    std::cout << "robot_legs_.size(): " << robot_legs_.size() << std::endl;

    // calculate total mass from urdf
    mass_ = 0;
    for (const auto &[fst, snd]: robot_tree.getSegments()) {
        mass_ += snd.segment.getInertia().getMass();
    }
}

std::vector<KDL::JntArray> QuadrupedRobot::getQ(const std::vector<KDL::Frame> &pEe_list) const {
    std::vector<KDL::JntArray> result;
    result.resize(4);
    for (int i(0); i < 4; ++i) {
        result[i] = robot_legs_[i]->calcQ(pEe_list[i], current_joint_pos_[i]);
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

KDL::Frame QuadrupedRobot::getFeet2BPositions(const int index) const {
    return robot_legs_[index]->calcPEe2B(current_joint_pos_[index]);
}

KDL::Jacobian QuadrupedRobot::getJacobian(const int index) const {
    return robot_legs_[index]->calcJaco(current_joint_pos_[index]);
}

KDL::JntArray QuadrupedRobot::getTorque(
    const KDL::Wrenches &force, const int index) const {
    return robot_legs_[index]->calcTorque(current_joint_pos_[index], current_joint_vel_[index], force);
}

void QuadrupedRobot::update(const CtrlComponent &ctrlComp) {
    for (int i = 0; i < 4; i++) {
        KDL::JntArray pos_array(3);
        pos_array(0) = ctrlComp.joint_position_state_interface_[i * 3].get().get_value();
        pos_array(1) = ctrlComp.joint_position_state_interface_[i * 3 + 1].get().get_value();
        pos_array(2) = ctrlComp.joint_position_state_interface_[i * 3 + 2].get().get_value();
        current_joint_pos_[i] = pos_array;

        KDL::JntArray vel_array(3);
        vel_array(0) = ctrlComp.joint_velocity_state_interface_[i * 3].get().get_value();
        vel_array(1) = ctrlComp.joint_velocity_state_interface_[i * 3 + 1].get().get_value();
        vel_array(2) = ctrlComp.joint_velocity_state_interface_[i * 3 + 2].get().get_value();
        current_joint_vel_[i] = vel_array;
    }
}
