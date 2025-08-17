//
// Created by biao on 24-9-14.
//
#pragma once


#include <memory>
#include <basic_quadruped_controller/common/mathTypes.h>
#include "LowPassFilter.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "QuadrupedKinematic.h"

struct CtrlInterfaces;
class WaveGenerator;
class QuadrupedKinematic;
struct CtrlComponent;

// Forward declarations for ROS2 types
namespace rclcpp_lifecycle
{
    class LifecycleNode;
}

class Estimator
{
public:
    explicit Estimator(CtrlInterfaces& ctrl_interfaces, CtrlComponent& ctrl_component,
                       std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    ~Estimator() = default;

    /**
     * Get the estimated robot central position
     * @return robot central position
     */
    Vec3 getPosition()
    {
        return x_hat_.segment(0, 3);
    }

    /**
     * Get the estimated robot central velocity
     * @return robot central velocity
     */
    Vec3 getVelocity()
    {
        return x_hat_.segment(3, 3);
    }

    /**
     * Get the estimated foot position in world frame
     * @param index leg index
     * @return foot position in world frame
     */
    Vec3 getFootPos(const int index)
    {
        return getPosition() + rotation_ * robot_model_->getFootPosition(index, FrameType::BODY);
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     */
    Vec34 getFeetPos()
    {
        Vec34 feet_pos;
        for (int i(0); i < 4; ++i)
        {
            feet_pos.col(i) = getFootPos(i);
        }
        return feet_pos;
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     */
    Vec34 getFeetVel()
    {
        Vec34 feet_vel = robot_model_->getFeet2BVelocities();
        for (int i(0); i < 4; ++i)
        {
            feet_vel.col(i) += getVelocity();
        }
        return feet_vel;
    };

    /**
     * Get the estimated foot position in body frame (similar to original estimator)
     * @return foot positions relative to body in global coordinate frame
     *
     * 重要说明：
     * - 返回的是相对于机体的足端位置，但方向已经转换到全局坐标系
     * - 这些位置用于状态估计，不是绝对的世界坐标
     * - 原版实现：只做旋转变换，不做位置偏移
     */
    Vec34 getFeetPos2Body()
    {
        // 将 (12,1) 向量转换为 (3,4) 矩阵格式
        Vec34 feet_pos_body_matrix;
        for (int i = 0; i < 4; ++i)
        {
            feet_pos_body_matrix.col(i) = feet_pos_body_.segment(3 * i, 3);
        }
        return feet_pos_body_matrix;
    }

    RotMat getRotation()
    {
        return rotation_;
    }

    Vec3 getGyro()
    {
        return gyro_;
    }

    [[nodiscard]] Vec3 getGyroGlobal() const
    {
        return rotation_ * gyro_;
    }

    [[nodiscard]] double getYaw() const;

    [[nodiscard]] double getDYaw() const
    {
        return getGyroGlobal()(2);
    }

    /**
     * Get foot positions in global coordinate frame (similar to original estimator)
     * @return foot positions in global coordinate frame
     */
    [[nodiscard]] Vec34 getFeetPosGlobal() const;

    /**
     * Get foot velocities in global coordinate frame (similar to original estimator)
     * @return foot velocities in global coordinate frame
     */
    [[nodiscard]] Vec34 getFeetVelGlobal() const;

    void update();

    /**
     * Publish odometry and tf data
     */
    void publishOdometryAndTf();

private:
    CtrlInterfaces& ctrl_interfaces_;
    std::shared_ptr<QuadrupedKinematic>& robot_model_;
    std::shared_ptr<WaveGenerator>& wave_generator_;

    Eigen::Matrix<double, 18, 1> x_hat_; // The state of estimator, position(3)+velocity(3)+feet position(3x4)

    Eigen::Matrix<double, 3, 1> u_; // The input of estimator

    Eigen::Matrix<double, 28, 1> y_; // The measurement value of output y
    Eigen::Matrix<double, 28, 1> y_hat_; // The prediction of output y
    Eigen::Matrix<double, 18, 18> A; // The transtion matrix of estimator
    Eigen::Matrix<double, 18, 3> B; // The input matrix
    Eigen::Matrix<double, 28, 18> C; // The output matrix

    // Covariance Matrix
    Eigen::Matrix<double, 18, 18> P; // Prediction covariance
    Eigen::Matrix<double, 18, 18> Ppriori; // Priori prediction covariance
    Eigen::Matrix<double, 18, 18> Q; // Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28> R; // Measurement covariance
    Eigen::Matrix<double, 18, 18> QInit_; // Initial value of Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28> RInit_; // Initial value of Measurement covariance
    Eigen::Matrix<double, 18, 1> Qdig; // adjustable process noise covariance
    Eigen::Matrix<double, 3, 3> Cu; // The covariance of system input u

    // Output Measurement
    Eigen::Matrix<double, 12, 1> feet_pos_body_; // The feet positions to body, in the global coordinate
    Eigen::Matrix<double, 12, 1> feet_vel_body_; // The feet velocity to body, in the global coordinate
    Eigen::Matrix<double, 4, 1> feet_h_; // The Height of each foot, in the global coordinate

    // Global coordinate foot positions (similar to original _feetPosGlobalKine)
    Vec34 feet_pos_global_kine_; // Foot positions in global coordinate frame
    Vec34 feet_vel_global_kine_; // Foot velocities in global coordinate frame
    Vec4 phase_;
    VecInt4 contact_;

    Eigen::Matrix<double, 28, 28> S; // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> Slu; // _S.lu()
    Eigen::Matrix<double, 28, 1> Sy; // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> Sc; // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> SR; // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> STC; // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> IKC; // _IKC = I - KC

    Vec3 g_;
    double dt_;

    RotMat rotation_;
    Vec3 acceleration_;
    Vec3 gyro_;

    std::vector<Vec3> foot_poses_;
    std::vector<Eigen::Vector3d> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter>> low_pass_filters_;

    double large_variance_;

    // ROS2 components for odometry and tf publishing
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Frame names
    std::string odom_frame_id_;
    std::string base_frame_id_;
};