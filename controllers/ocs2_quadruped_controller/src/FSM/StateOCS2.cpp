//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        if (!node_->has_parameter("default_kp"))
        {
            node_->declare_parameter("default_kp", default_kp_);
        }
        if (!node_->has_parameter("default_kd"))
        {
            node_->declare_parameter("default_kd", default_kd_);
        }
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::enter()
    {
        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& /**time**/,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        for (int i = 0; i < 12; i++)
        {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                             ctrl_component_->mpc_mrt_interface_->getCommand());
    }

    void StateOCS2::exit()
    {
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }
}
