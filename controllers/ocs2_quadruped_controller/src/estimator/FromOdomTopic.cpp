//
// Created by biao on 25-2-23.
//

#include "ocs2_quadruped_controller/estimator/FromOdomTopic.h"

namespace ocs2::legged_robot
{
    FromOdomTopic::FromOdomTopic(CentroidalModelInfo info, CtrlComponent& ctrl_component,
                                 const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) :
        StateEstimateBase(
            std::move(info), ctrl_component,
            node)
    {
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                // Handle message
                position_ = {
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z,
                };

                linear_velocity_ = {
                    msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y,
                    msg->twist.twist.linear.z,
                };
            });
    }

    vector_t FromOdomTopic::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        updateJointStates();
        updateImu();

        updateLinear(position_, linear_velocity_);
        return rbd_state_;
    }
}
