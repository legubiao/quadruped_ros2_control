//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <tf2_ros/transform_broadcaster.h>

namespace ocs2::legged_robot {
    class KalmanFilterEstimate final : public StateEstimateBase {
    public:
        KalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &ee_kinematics,
                             CtrlInterfaces &ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;

        vector_t feet_heights_;

        // Config
        scalar_t foot_radius_ = 0.02;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

    private:
        size_t numContacts_, dimContacts_, numState_, numObserve_;

        matrix_t a_, b_, c_, q_, p_, r_;
        vector_t xHat_, ps_, vs_;
    };
}
