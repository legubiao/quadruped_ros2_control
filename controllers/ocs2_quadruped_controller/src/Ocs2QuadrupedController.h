//
// Created by tlab-uav on 24-9-24.
//

#ifndef OCS2QUADRUPEDCONTROLLER_H
#define OCS2QUADRUPEDCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_quadruped_controller/estimator/StateEstimateBase.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include "SafetyChecker.h"
#include "ocs2_quadruped_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot {
    class Ocs2QuadrupedController final : public controller_interface::ControllerInterface {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        Ocs2QuadrupedController() = default;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        void setupLeggedInterface();

        void setupMpc();

        void setupMrt();

        void setupStateEstimate();

        CtrlComponent ctrl_comp_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_comp_.joint_torque_command_interface_},
            {"position", &ctrl_comp_.joint_position_command_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_command_interface_},
            {"kp", &ctrl_comp_.joint_kp_command_interface_},
            {"kd", &ctrl_comp_.joint_kd_command_interface_}
        };
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_comp_.joint_position_state_interface_},
            {"effort", &ctrl_comp_.joint_effort_state_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_state_interface_}
        };

        // IMU Sensor
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;
        // Foot Force Sensor
        std::string foot_force_name_;
        std::vector<std::string> foot_force_interface_types_;

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;


        SystemObservation currentObservation_;

        std::string task_file_;
        std::string urdf_file_;
        std::string reference_file_;

        bool verbose_;

        std::shared_ptr<LeggedInterface> legged_interface_;
        std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safetyChecker_;

        // Nonlinear MPC
        std::shared_ptr<MPC_BASE> mpc_;
        std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

        std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
        std::shared_ptr<StateEstimateBase> stateEstimate_;

    private:
        std::thread mpcThread_;
        std::atomic_bool controllerRunning_{}, mpcRunning_{};
        benchmark::RepeatedTimer mpcTimer_;
        benchmark::RepeatedTimer wbcTimer_;
    };
}

#endif //OCS2QUADRUPEDCONTROLLER_H
