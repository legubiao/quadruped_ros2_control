//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <unitree_guide_controller/gait/WaveGenerator.h>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    imu_state_interface_;

    control_input_msgs::msg::Inputs control_inputs_;
    int frequency_{};

    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;

    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;

    void clear() {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
    }
};

#endif //INTERFACE_H
