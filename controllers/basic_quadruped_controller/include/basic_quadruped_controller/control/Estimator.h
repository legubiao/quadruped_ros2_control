//
// Created by biao on 24-9-14.
//

#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <memory>
#include <kdl/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>
#include <basic_quadruped_controller/common/mathTypes.h>
#include "LowPassFilter.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

struct CtrlInterfaces;
class WaveGenerator;
class QuadrupedKinematic;
struct CtrlComponent;

// Forward declarations for ROS2 types
namespace rclcpp_lifecycle {
    class LifecycleNode;
}

class Estimator {
public:
    explicit Estimator(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component, 
                      std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    ~Estimator() = default;

    /**
     * Get the estimated robot central position
     * @return robot central position
     */
    Vec3 getPosition() {
        return x_hat_.segment(0, 3);
    }

    /**
     * Get the estimated robot central velocity
     * @return robot central velocity
     */
    Vec3 getVelocity() {
        // Return filtered velocity for stability (as per original unitree implementation)
        Vec3 filtered_vel;
        filtered_vel(0) = low_pass_filters_[0]->getValue();
        filtered_vel(1) = low_pass_filters_[1]->getValue(); 
        filtered_vel(2) = low_pass_filters_[2]->getValue();
        return filtered_vel;
    }

    /**
     * Get the estimated foot position in world frame
     * @param index leg index
     * @return foot position in world frame
     */
    Vec3 getFootPos(const int index) {
        return getPosition() + rotation_ * foot_poses_[index].translation();
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     */
    Vec34 getFeetPos() {
        Vec34 feet_pos;
        for (int i(0); i < 4; ++i) {
            feet_pos.col(i) = getFootPos(i);
        }
        return feet_pos;
    }

    /**
     * Get the estimated feet velocity in world frame
     * @return feet velocity in world frame
     */
    Vec34 getFeetVel();

    /**
     * Get the estimated foot position in body frame
     * @return
     */
    Vec34 getFeetPos2Body() {
        Vec34 foot_pos;
        const Vec3 body_pos = getPosition();
        for (int i = 0; i < 4; i++) {
            foot_pos.col(i) = getFootPos(i) - body_pos;
        }
        return foot_pos;
    }

    RotMat getRotation() {
        return rotation_;
    }

    Vec3 getGyro() {
        return gyro_;
    }

    [[nodiscard]] Vec3 getGyroGlobal() const {
        return rotation_ * gyro_;
    }

    [[nodiscard]] double getYaw() const;

    [[nodiscard]] double getDYaw() const {
        return getGyroGlobal()(2);
    }

    void update();

    /**
     * Publish odometry and tf data
     */
    void publishOdometryAndTf();

private:
    CtrlInterfaces &ctrl_interfaces_;
    std::shared_ptr<QuadrupedKinematic> &robot_model_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

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

    Eigen::Matrix<double, 28, 28> S; // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28> > Slu; // _S.lu()
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

    std::vector<pinocchio::SE3> foot_poses_;
    std::vector<Eigen::Vector3d> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter> > low_pass_filters_;

    double large_variance_;

    // ROS2 components for odometry and tf publishing
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Frame names
    std::string odom_frame_id_;
    std::string base_frame_id_;
};


#endif //ESTIMATOR_H
