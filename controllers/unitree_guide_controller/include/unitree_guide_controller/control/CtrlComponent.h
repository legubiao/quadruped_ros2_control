//
// Created by biao on 24-9-10.
//

#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> >
    joint_effort_command_interface_;
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

    control_input_msgs::msg::Inputs default_inputs_;
    std::reference_wrapper<control_input_msgs::msg::Inputs> control_inputs_;
    int frequency_{};

    QuadrupedRobot default_robot_model_;
    std::reference_wrapper<QuadrupedRobot> robot_model_;

    Estimator default_estimator_;
    std::reference_wrapper<Estimator> estimator_;

    BalanceCtrl default_balance_ctrl_;
    std::reference_wrapper<BalanceCtrl> balance_ctrl_;

    CtrlComponent() : control_inputs_(default_inputs_), robot_model_(default_robot_model_),
                      estimator_(default_estimator_), balance_ctrl_(default_balance_ctrl_) {
    }

    void clear() {
        joint_effort_command_interface_.clear();
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
