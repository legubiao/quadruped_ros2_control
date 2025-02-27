//
// Created by qiayuan on 2022/7/24.
//
#pragma once

#include "StateEstimateBase.h"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <tf2_ros/transform_broadcaster.h>

namespace ocs2::legged_robot {
    class GroundTruth final : public StateEstimateBase {
    public:
        GroundTruth(CentroidalModelInfo info, CtrlInterfaces &ctrl_component,
                    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        nav_msgs::msg::Odometry getOdomMsg();
        vector3_t position_;
        vector3_t linear_velocity_;
    };
}
