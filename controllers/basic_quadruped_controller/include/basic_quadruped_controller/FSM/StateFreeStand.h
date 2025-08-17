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

    Vec3 fr_init_pos_;           // 初始机身位置（类似_initVecOX）
    Vec34 init_foot_pos_;    // 初始足端位置（类似_initVecXP）
    
    // 私有方法
    void calc_body_target(double row, double pitch, double yaw, double height);
    Vec34 calcOP(double row, double pitch, double yaw, double height);  // 对应原版_calcOP
    

};

#endif //STATEFREESTAND_H
