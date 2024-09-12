//
// Created by biao on 24-9-12.
//


#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H
#include <string>
#include <kdl_parser/kdl_parser/kdl_parser.hpp>

#include "RobotLeg.h"


class QuadrupedRobot {
public:
    explicit QuadrupedRobot(const std::string &robot_description);

    /**
     * Calculate the joint positions based on the foot end position
     * @param pEe_list vector of foot-end position
     * @param q_init vector of current joint positions
     * @return
     */
    [[nodiscard]] std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list,
                                                  const std::vector<KDL::JntArray> &q_init) const;

    /**
     * Calculate the foot end position based on joint positions
     * @param joint_positions vector of current joint positions
     * @return vector of foot-end position
     */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions(const std::vector<KDL::JntArray> &joint_positions) const;

protected:
    double mass_;

    std::vector<Robotleg> robot_legs_;

    KDL::Chain fr_chain_;
    KDL::Chain fl_chain_;
    KDL::Chain rr_chain_;
    KDL::Chain rl_chain_;
};


#endif //QUADRUPEDROBOT_H
