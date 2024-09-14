//
// Created by biao on 24-9-12.
//


#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H
#include <string>
#include <kdl_parser/kdl_parser.hpp>

#include "RobotLeg.h"


struct CtrlComponent;

class QuadrupedRobot {
public:
    explicit QuadrupedRobot() = default;

    ~QuadrupedRobot() = default;

    void init(const std::string &robot_description);

    /**
     * Calculate the joint positions based on the foot end position
     * @param pEe_list vector of foot-end position
     * @return
     */
    [[nodiscard]] std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list) const;

    /**
     * Calculate the foot end position based on joint positions
     * @param joint_positions vector of joint positions
     * @return vector of foot-end position
     */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions(const std::vector<KDL::JntArray> &joint_positions) const;

    /**
     * Calculate the foot end position based on joint positions
     * @param index leg index
     * @return foot-end position
     */
    [[nodiscard]] KDL::Frame getFeet2BPositions(int index) const;

    /**
     * Calculate the Jacobian matrix based on joint positions
     * @param index leg index
     * @return Jacobian matrix
     */
    [[nodiscard]] KDL::Jacobian getJacobian(int index) const;

    /**
     * Calculate the torque based on joint positions, joint velocities and external force
     * @param force external force
     * @param index leg index
     * @return torque
     */
    [[nodiscard]] KDL::JntArray getTorque(
        const KDL::Wrenches &force, int index) const;

    std::vector<KDL::JntArray> current_joint_pos_;
    std::vector<KDL::JntArray> current_joint_vel_;

    void update(const CtrlComponent &ctrlComp);

private:
    double mass_ = 0;

    std::vector<std::shared_ptr<RobotLeg> > robot_legs_;

    KDL::Chain fr_chain_;
    KDL::Chain fl_chain_;
    KDL::Chain rr_chain_;
    KDL::Chain rl_chain_;
};


#endif //QUADRUPEDROBOT_H
