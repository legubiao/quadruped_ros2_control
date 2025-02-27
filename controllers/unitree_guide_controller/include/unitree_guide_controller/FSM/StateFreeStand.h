//
// Created by tlab-uav on 24-9-13.
//

#ifndef STATEFREESTAND_H
#define STATEFREESTAND_H
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "controller_common/FSM/FSMState.h"

struct CtrlComponent;

class StateFreeStand final : public FSMState {
public:
    explicit StateFreeStand(CtrlInterfaces &ctrl_interfaces,
                            CtrlComponent &ctrl_component);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    std::shared_ptr<QuadrupedRobot> &robot_model_;

    void calc_body_target(float row, float pitch, float yaw, float height);

    float row_max_, row_min_;
    float pitch_max_, pitch_min_;
    float yaw_max_, yaw_min_;
    float height_max_, height_min_;

    std::vector<KDL::JntArray> init_joint_pos_;
    std::vector<KDL::JntArray> target_joint_pos_;

    KDL::Frame fr_init_pos_;
    std::vector<KDL::Frame> init_foot_pos_; // 4 feet position in fr-foot frame
};

#endif //STATEFREESTAND_H
