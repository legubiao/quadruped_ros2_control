//
// Created by biao on 25-2-23.
//

#include "ocs2_quadruped_controller/estimator/FromOdomTopic.h"

namespace ocs2::legged_robot {
    FromOdomTopic::FromOdomTopic(CentroidalModelInfo info, CtrlInterfaces &ctrl_component,
                                 const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) : StateEstimateBase(
        std::move(info), ctrl_component,
        node) {
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                buffer_.writeFromNonRT(*msg);
            });
    }

    vector_t FromOdomTopic::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        const nav_msgs::msg::Odometry odom = *buffer_.readFromRT();

        updateJointStates();
        updateAngular(quatToZyx(Eigen::Quaternion(
                          odom.pose.pose.orientation.w,
                          odom.pose.pose.orientation.x,
                          odom.pose.pose.orientation.y,
                          odom.pose.pose.orientation.z)),
                      Eigen::Matrix<scalar_t, 3, 1>(
                          odom.twist.twist.angular.x,
                          odom.twist.twist.angular.y,
                          odom.twist.twist.angular.z));
        updateLinear(Eigen::Matrix<scalar_t, 3, 1>(
                         odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z),
                     Eigen::Matrix<scalar_t, 3, 1>(
                         odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         odom.twist.twist.linear.z));
        return rbd_state_;
    }
}
