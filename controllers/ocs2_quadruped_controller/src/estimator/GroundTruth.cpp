//
// Created by qiayuan on 2022/7/24.
//

#include "ocs2_quadruped_controller/estimator/GroundTruth.h"

namespace ocs2::legged_robot
{
    GroundTruth::GroundTruth(CentroidalModelInfo info, CtrlInterfaces& ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
        : StateEstimateBase(
            std::move(info), ctrl_component,
            node)
    {
        initPublishers();
    }

    vector_t GroundTruth::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        updateJointStates();
        updateImu();

        position_ = {
            ctrl_component_.odom_state_interface_[0].get().get_value(),
            ctrl_component_.odom_state_interface_[1].get().get_value(),
            ctrl_component_.odom_state_interface_[2].get().get_value()
        };

        linear_velocity_ = {
            ctrl_component_.odom_state_interface_[3].get().get_value(),
            ctrl_component_.odom_state_interface_[4].get().get_value(),
            ctrl_component_.odom_state_interface_[5].get().get_value()
        };

        updateLinear(position_, linear_velocity_);

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        return rbd_state_;
    }

    nav_msgs::msg::Odometry GroundTruth::getOdomMsg()
    {
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = position_(0);
        odom.pose.pose.position.y = position_(1);
        odom.pose.pose.position.z = position_(2);
        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();

        odom.twist.twist.linear.x = linear_velocity_(0);
        odom.twist.twist.linear.y = linear_velocity_(1);
        odom.twist.twist.linear.z = linear_velocity_(2);
        odom.twist.twist.angular.x = angular_vel_local_.x();
        odom.twist.twist.angular.y = angular_vel_local_.y();
        odom.twist.twist.angular.z = angular_vel_local_.z();

        return odom;
    }
} // namespace legged
