//
// Created by biao on 25-2-23.
//

#pragma once
#include "StateEstimateBase.h"
#include <realtime_tools/realtime_tools/realtime_buffer.hpp>

namespace ocs2::legged_robot
{
    class FromOdomTopic final : public StateEstimateBase
    {
    public:
        FromOdomTopic(CentroidalModelInfo info, CtrlInterfaces& ctrl_component,
                      const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

        vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    protected:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
    };
};
