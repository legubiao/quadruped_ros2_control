//
// Created by tlab-uav on 24-9-13.
//

#ifndef STATEFREESTAND_H
#define STATEFREESTAND_H
#include <basic_quadruped_controller/control/QuadrupedKinematic.h>

#include "controller_common/FSM/FSMState.h"

struct CtrlComponent;

class StateFreeStand final : public FSMState {
public:
    StateFreeStand(CtrlInterfaces &ctrl_interfaces,
                   CtrlComponent &ctrl_component,
                   const double kp,
                   const double kd);

    void enter() override;

    void run(const rclcpp::Time &time,
             const rclcpp::Duration &period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    std::shared_ptr<QuadrupedKinematic> &robot_model_;

    // 控制参数
    double kp_, kd_;

    // 姿态限制
    double row_max_, row_min_;
    double pitch_max_, pitch_min_;
    double yaw_max_, yaw_min_;
    double height_max_, height_min_;

    Vec12 init_joint_pos_;
    Vec12 target_joint_pos_;

    pinocchio::SE3 fl_init_pos_;
    std::vector<pinocchio::SE3> init_foot_pos_; // 4 feet position in fl-foot frame
    
    // 私有方法
    void calc_body_target(const float row, const float pitch, const float yaw, const float height);
};

#endif //STATEFREESTAND_H
