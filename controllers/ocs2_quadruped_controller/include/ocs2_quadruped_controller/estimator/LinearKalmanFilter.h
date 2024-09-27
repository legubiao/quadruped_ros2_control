//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ocs2::legged_robot {
    class KalmanFilterEstimate final : public StateEstimateBase {
    public:
        KalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &ee_kinematics,
                             CtrlComponent &ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &taskFile, bool verbose);

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        vector_t feetHeights_;

        // Config
        scalar_t footRadius_ = 0.02;
        scalar_t imuProcessNoisePosition_ = 0.02;
        scalar_t imuProcessNoiseVelocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

    private:
        size_t numContacts_, dimContacts_, numState_, numObserve_;

        matrix_t a_, b_, c_, q_, p_, r_;
        vector_t xHat_, ps_, vs_;
    };
} // namespace legged
