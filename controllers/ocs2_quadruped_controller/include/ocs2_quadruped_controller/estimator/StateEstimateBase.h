//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ocs2::legged_robot
{
    class StateEstimateBase
    {
    public:
        virtual ~StateEstimateBase() = default;

        StateEstimateBase(CentroidalModelInfo info, CtrlInterfaces& ctrl_component,
                          rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        virtual void updateJointStates();

        virtual void updateContact();

        virtual void updateImu();

        virtual vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;

        [[nodiscard]] size_t getMode() const { return stanceLeg2ModeNumber(contact_flag_); }

    protected:
        void initPublishers();

        void updateAngular(const vector3_t& zyx, const vector_t& angularVel);

        void updateLinear(const vector_t& pos, const vector_t& linearVel);

        void publishMsgs(const nav_msgs::msg::Odometry& odom) const;

        CtrlInterfaces& ctrl_component_;
        CentroidalModelInfo info_;

        contact_flag_t contact_flag_{};
        double feet_force_threshold_ = 5.0;

        vector3_t zyx_offset_ = vector3_t::Zero();
        vector_t rbd_state_;
        Eigen::Quaternion<scalar_t> quat_;
        vector3_t angular_vel_local_, linear_accel_local_;
        matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };

    template <typename T>
    T square(T a)
    {
        return a * a;
    }

    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q)
    {
        Eigen::Matrix<SCALAR_T, 3, 1> zyx;

        SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
        zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                            square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
        zyx(1) = std::asin(as);
        zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                            square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
        return zyx;
    }
} // namespace legged
