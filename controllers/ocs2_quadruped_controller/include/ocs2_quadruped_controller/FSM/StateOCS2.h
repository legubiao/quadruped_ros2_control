//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/detail/mpc_observation__struct.hpp>
#include <ocs2_quadruped_controller/control/TargetManager.h>
#include <ocs2_quadruped_controller/estimator/StateEstimateBase.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "controller_common/FSM/FSMState.h"

namespace ocs2 {
    class MPC_MRT_Interface;
    class MPC_BASE;
    class PinocchioEndEffectorKinematics;
}

struct CtrlComponent;

namespace ocs2::legged_robot {
    class StateOCS2 final : public FSMState {
    public:
        StateOCS2(CtrlInterfaces &ctrl_interfaces,
                  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                  const std::string &package_share_directory,
                  const std::vector<std::string> &joint_names,
                  const std::vector<std::string> &feet_names,
                  double default_kp,
                  double default_kd
        );

        void enter() override;

        void run(const rclcpp::Time &time,
                 const rclcpp::Duration &period) override;

        void exit() override;

        FSMStateName checkChange() override;

        void setupStateEstimate(const std::string &estimator_type);

    private:
        void updateStateEstimation(const rclcpp::Duration &period);

        void setupLeggedInterface();

        void setupMpc();

        void setupMrt();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;
        std::shared_ptr<StateEstimateBase> estimator_;
        std::shared_ptr<TargetManager> target_manager_;
        std::shared_ptr<LeggedRobotVisualizer> visualizer_;

        std::shared_ptr<LeggedInterface> legged_interface_;
        std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;

        // Nonlinear MPC
        std::shared_ptr<MPC_BASE> mpc_;
        std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

        std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;

        SystemObservation observation_;

        vector_t measured_rbd_state_;
        std::thread mpc_thread_;
        std::atomic_bool controller_running_{}, mpc_running_{};
        benchmark::RepeatedTimer mpc_timer_;
        benchmark::RepeatedTimer wbc_timer_;

        std::string task_file_;
        std::string urdf_file_;
        std::string reference_file_;
        std::string gait_file_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> feet_names_;
        double default_kp_;
        double default_kd_;

        bool verbose_;
        bool mpc_need_updated_;
        vector_t optimized_state_, optimized_input_;
    };
}


#endif //STATEOCS2_H
