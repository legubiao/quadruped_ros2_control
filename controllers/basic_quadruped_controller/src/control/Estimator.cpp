//
// Created by biao on 24-9-14.
//

#include "basic_quadruped_controller/control/Estimator.h"

#include <basic_quadruped_controller/common/mathTools.h>
#include <basic_quadruped_controller/control/CtrlComponent.h>
#include <pinocchio/spatial/se3.hpp>

#include "controller_common/CtrlInterfaces.h"

Estimator::Estimator(CtrlInterfaces &ctrl_interfaces, CtrlComponent &ctrl_component,
                     std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
    : ctrl_interfaces_(ctrl_interfaces),
      robot_model_(ctrl_component.robot_model_),
      wave_generator_(ctrl_component.wave_generator_),
      node_(node) {
    g_ << 0, 0, -9.81;
    dt_ = 1.0 / ctrl_interfaces.frequency_;

    std::cout << "dt: " << dt_ << std::endl;
    large_variance_ = 100;
    for (int i(0); i < Qdig.rows(); ++i) {
        Qdig(i) = i < 6 ? 0.0003 : 0.01;
    }

    x_hat_.setZero();
    u_.setZero();

    A.setZero();
    A.block(0, 0, 3, 3) = I3();
    A.block(0, 3, 3, 3) = I3() * dt_;
    A.block(3, 3, 3, 3) = I3();
    A.block(6, 6, 12, 12) = I12;

    B.setZero();
    B.block(3, 0, 3, 3) = I3() * dt_;

    C.setZero();
    C.block(0, 0, 3, 3) = -I3();
    C.block(3, 0, 3, 3) = -I3();
    C.block(6, 0, 3, 3) = -I3();
    C.block(9, 0, 3, 3) = -I3();
    C.block(12, 3, 3, 3) = -I3();
    C.block(15, 3, 3, 3) = -I3();
    C.block(18, 3, 3, 3) = -I3();
    C.block(21, 3, 3, 3) = -I3();
    C.block(0, 6, 12, 12) = I12;
    C(24, 8) = 1;
    C(25, 11) = 1;
    C(26, 14) = 1;
    C(27, 17) = 1;

    P.setIdentity();
    P = large_variance_ * P;

    RInit_ << 0.008, 0.012, -0.000, -0.009, 0.012, 0.000, 0.009, -0.009, -0.000,
            -0.009, -0.009, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, -0.001,
            -0.002, 0.000, -0.000, -0.003, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            0.012, 0.019, -0.001, -0.014, 0.018, -0.000, 0.014, -0.013, -0.000,
            -0.014, -0.014, 0.001, -0.001, 0.001, -0.001, 0.000, 0.000, -0.001,
            -0.003, 0.000, -0.001, -0.004, -0.000, -0.001, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, 0.001, 0.001, -0.001, 0.000, -0.000, 0.000, -0.000, 0.001,
            0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000,
            -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, -0.009, -0.014,
            0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010, -0.000,
            0.001, 0.000, 0.000, 0.001, -0.000, 0.001, 0.002, -0.000, 0.000, 0.003,
            0.000, 0.001, 0.000, 0.000, 0.000, 0.000, 0.012, 0.018, -0.001, -0.013,
            0.018, -0.000, 0.013, -0.013, -0.000, -0.013, -0.013, 0.001, -0.001,
            0.000, -0.001, 0.000, 0.001, -0.001, -0.003, 0.000, -0.001, -0.004,
            -0.000, -0.001, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
            -0.000, 0.001, 0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000,
            -0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.009, 0.014, -0.000, -0.010, 0.013, 0.000,
            0.010, -0.010, -0.000, -0.010, -0.010, 0.000, -0.001, 0.000, -0.001,
            0.000, -0.000, -0.001, -0.001, 0.000, -0.000, -0.003, -0.000, -0.001,
            0.000, 0.000, 0.000, 0.000, -0.009, -0.013, 0.000, 0.010, -0.013, 0.000,
            -0.010, 0.009, 0.000, 0.010, 0.010, -0.000, 0.001, -0.000, 0.000, -0.000,
            0.000, 0.001, 0.002, 0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000,
            0.000, 0.000, -0.000, -0.000, -0.000, 0.000, -0.000, -0.000, -0.000,
            0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000,
            -0.000, 0.000, -0.000, 0.000, 0.000, -0.000, -0.000, 0.000, 0.000, 0.000,
            0.000, -0.009, -0.014, 0.001, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000,
            0.010, 0.010, -0.000, 0.001, 0.000, 0.000, -0.000, -0.000, 0.001, 0.002,
            -0.000, 0.000, 0.003, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, -0.009,
            -0.014, 0.000, 0.010, -0.013, 0.000, -0.010, 0.010, 0.000, 0.010, 0.010,
            -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, 0.001, 0.002, -0.000, 0.000,
            0.003, 0.001, 0.001, 0.000, 0.000, 0.000, 0.000, 0.000, 0.001, -0.000,
            -0.000, 0.001, -0.000, 0.000, -0.000, 0.000, -0.000, -0.000, 0.001, 0.000,
            -0.000, -0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.001, 0.000, 0.001, -0.001,
            -0.000, -0.001, 0.001, 0.000, 0.001, 0.001, 0.000, 1.708, 0.048, 0.784,
            0.062, 0.042, 0.053, 0.077, 0.001, -0.061, 0.046, -0.019, -0.029, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.001, -0.000, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.000, 0.000, -0.000, -0.000, 0.048, 5.001, -1.631, -0.036,
            0.144, 0.040, 0.036, 0.016, -0.051, -0.067, -0.024, -0.005, 0.000, 0.000,
            0.000, 0.000, -0.000, -0.001, 0.000, 0.000, -0.001, -0.000, -0.001, 0.000,
            0.000, 0.000, 0.000, -0.000, 0.784, -1.631, 1.242, 0.057, -0.037, 0.018,
            0.034, -0.017, -0.015, 0.058, -0.021, -0.029, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000,
            -0.000, -0.000, 0.062, -0.036, 0.057, 6.228, -0.014, 0.932, 0.059, 0.053,
            -0.069, 0.148, 0.015, -0.031, 0.000, 0.000, 0.000, 0.000, -0.000, 0.000,
            -0.000, -0.000, 0.001, 0.000, -0.000, 0.000, 0.000, -0.000, 0.000, 0.000,
            0.042, 0.144, -0.037, -0.014, 3.011, 0.986, 0.076, 0.030, -0.052, -0.027,
            0.057, 0.051, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, -0.000, 0.001,
            -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, 0.053, 0.040,
            0.018, 0.932, 0.986, 0.885, 0.090, 0.044, -0.055, 0.057, 0.051, -0.003,
            0.000, 0.000, 0.000, 0.000, -0.002, -0.003, 0.000, 0.002, -0.003, -0.000,
            -0.001, 0.002, 0.000, 0.002, 0.002, -0.000, 0.077, 0.036, 0.034, 0.059,
            0.076, 0.090, 6.230, 0.139, 0.763, 0.013, -0.019, -0.024, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, -0.000, -0.000, 0.000, -0.000, 0.000, 0.000,
            -0.000, -0.000, -0.000, 0.000, 0.001, 0.016, -0.017, 0.053, 0.030, 0.044,
            0.139, 3.130, -1.128, -0.010, 0.131, 0.018, 0.000, 0.000, 0.000, 0.000,
            -0.000, -0.001, -0.000, 0.000, -0.001, -0.000, -0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,
            0.763, -1.128, 0.866, -0.022, -0.053, 0.007, 0.000, 0.000, 0.000, 0.000,
            -0.003, -0.004, -0.000, 0.003, -0.004, -0.000, -0.003, 0.003, 0.000,
            0.003, 0.003, 0.000, 0.046, -0.067, 0.058, 0.148, -0.027, 0.057, 0.013,
            -0.010, -0.022, 2.437, -0.102, 0.938, 0.000, 0.000, 0.000, 0.000, -0.000,
            -0.000, 0.000, 0.000, -0.000, 0.000, -0.000, 0.000, -0.000, 0.000, 0.001,
            0.000, -0.019, -0.024, -0.021, 0.015, 0.057, 0.051, -0.019, 0.131, -0.053,
            -0.102, 4.944, 1.724, 0.000, 0.000, 0.000, 0.000, -0.001, -0.001, 0.000,
            0.001, -0.001, 0.000, -0.001, 0.001, -0.000, 0.001, 0.001, 0.000, -0.029,
            -0.005, -0.029, -0.031, 0.051, -0.003, -0.024, 0.018, 0.007, 0.938, 1.724,
            1.569, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 1.0, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
            0.000, 0.000, 0.000, 1.0;

    Cu << 268.573, -43.819, -147.211, -43.819, 92.949, 58.082, -147.211, 58.082,
            302.120;

    QInit_ = Qdig.asDiagonal();
    QInit_ += B * Cu * B.transpose();

    low_pass_filters_.resize(3);
    low_pass_filters_[0] = std::make_shared<LowPassFilter>(dt_, 3.0);
    low_pass_filters_[1] = std::make_shared<LowPassFilter>(dt_, 3.0);
    low_pass_filters_[2] = std::make_shared<LowPassFilter>(dt_, 3.0);

    // Initialize ROS2 components
    odom_frame_id_ = "leg_odom";
    base_frame_id_ = "base";
    
    // Create odometry publisher
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Create tf broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

double Estimator::getYaw() const {
    return rotMatToRPY(rotation_)(2);
}

Vec34 Estimator::getFeetVel() {
    const std::vector<Eigen::Vector3d> feet_vel = robot_model_->getFeet2BVelocities();
    Vec34 result;
    for (int i(0); i < 4; ++i) {
        result.col(i) = feet_vel[i] + getVelocity();
    }
    return result;
}

void Estimator::update() {

    if (robot_model_->mass_ == 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                            "Robot mass is 0, skipping estimation update");
        return;
    }

    // Check if we have valid sensor data before proceeding
    if (!ctrl_interfaces_.imu_state_interface_.empty()) {
        // Check if IMU data is valid (not NaN or extremely large)
        double imu_accel_x = ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value_or(0.0);
        double imu_accel_y = ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value_or(0.0);
        double imu_accel_z = ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value_or(0.0);
        
        if (!std::isfinite(imu_accel_x) || !std::isfinite(imu_accel_y) || !std::isfinite(imu_accel_z) ||
            std::abs(imu_accel_x) > 1000 || std::abs(imu_accel_y) > 1000 || std::abs(imu_accel_z) > 1000) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                "Invalid IMU data detected, skipping estimation update");
            return;
        }
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                            "No IMU data available, skipping estimation update");
        return;
    }

    // Check if joint data is available
    if (ctrl_interfaces_.joint_position_state_interface_.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                            "No joint data available, skipping estimation update");
        return;
    }



    Q = QInit_;
    R = RInit_;

    foot_poses_ = robot_model_->getFeet2BPositions();
    foot_vels_ = robot_model_->getFeet2BVelocities();
    
    // In SWING_ALL mode, set foot velocities to zero to avoid numerical issues
    if (wave_generator_->status_ == WaveStatus::SWING_ALL) {
        for (int i = 0; i < 4; ++i) {
            foot_vels_[i].setZero();
        }
    }
    
    feet_h_.setZero();

    // Adjust the covariance based on foot contact and phase.
    for (int i(0); i < 4; ++i) {
        if (wave_generator_->contact_[i] == 0) {
            // foot not contact
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
            R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = large_variance_ * Eigen::MatrixXd::Identity(3, 3);
            R(24 + i, 24 + i) = large_variance_;
            

        } else {
            // foot contact
            const double trust = windowFunc(wave_generator_->phase_[i], 0.2);
            double pos_variance_factor = (1 + (1 - trust) * large_variance_);
            double vel_variance_factor = (1 + (1 - trust) * large_variance_);
            double height_variance_factor = (1 + (1 - trust) * large_variance_);
            
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = pos_variance_factor * QInit_.block(6 + 3 * i, 6 + 3 * i, 3, 3);
            R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = vel_variance_factor * RInit_.block(12 + 3 * i, 12 + 3 * i, 3, 3);
            R(24 + i, 24 + i) = height_variance_factor * RInit_(24 + i, 24 + i);
        }
        // For non-contact feet, set observation to zero to avoid numerical issues
        if (wave_generator_->contact_[i] == 0) {
            feet_pos_body_.segment(3 * i, 3).setZero();
            feet_vel_body_.segment(3 * i, 3).setZero();
        } else {
            feet_pos_body_.segment(3 * i, 3) = foot_poses_[i].translation();
            feet_vel_body_.segment(3 * i, 3) = foot_vels_[i];
        }
    }

    Quat quat;
    quat << ctrl_interfaces_.imu_state_interface_[0].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[1].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[2].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[3].get().get_optional().value();
    rotation_ = quatToRotMat(quat);

    gyro_ << ctrl_interfaces_.imu_state_interface_[4].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[5].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[6].get().get_optional().value();

    acceleration_ << ctrl_interfaces_.imu_state_interface_[7].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[8].get().get_optional().value(),
            ctrl_interfaces_.imu_state_interface_[9].get().get_optional().value();

    u_ = rotation_ * acceleration_ + g_;
    x_hat_ = A * x_hat_ + B * u_;
    y_hat_ = C * x_hat_;

    // Update the measurement value
    y_ << feet_pos_body_, feet_vel_body_, feet_h_;

    // Update the covariance matrix
    Ppriori = A * P * A.transpose() + Q;
    S = R + C * Ppriori * C.transpose();
    Slu = S.lu();
    Sy = Slu.solve(y_ - y_hat_);
    Sc = Slu.solve(C);
    SR = Slu.solve(R);
    STC = S.transpose().lu().solve(C);
    IKC = Eigen::MatrixXd::Identity(18, 18) - Ppriori * C.transpose() * Sc;

    // Update the state and covariance matrix
    x_hat_ += Ppriori * C.transpose() * Sy;
    P = IKC * Ppriori * IKC.transpose() +
        Ppriori * C.transpose() * SR * STC * Ppriori.transpose();

    // // Using low pass filter to smooth the velocity
    // Apply low-pass filter to velocity estimates (for external use, not modifying x_hat_)
    low_pass_filters_[0]->addValue(x_hat_(3));
    low_pass_filters_[1]->addValue(x_hat_(4));
    low_pass_filters_[2]->addValue(x_hat_(5));
    // Note: Do NOT write filtered values back to x_hat_ as it breaks Kalman filter consistency!
    
    // Check for numerical divergence and reset if necessary
    bool diverged = false;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(x_hat_(i)) || std::abs(x_hat_(i)) > 1e6) {
            diverged = true;
            break;
        }
    }
    
    if (diverged) {
        RCLCPP_WARN(node_->get_logger(), "Estimator state diverged - resetting to zero");
        
        x_hat_.setZero();
        P.setIdentity();
        P = large_variance_ * P;
        // Reset low pass filters
        for (auto& filter : low_pass_filters_) {
            filter->clear();
        }
        return; // Skip publishing this invalid state
    }
    
    // Publish odometry and tf
    publishOdometryAndTf();
}

void Estimator::publishOdometryAndTf() {
    // Get current time
    rclcpp::Time current_time = node_->get_clock()->now();
    
    // Get estimated position and velocity
    Vec3 position = getPosition();
    Vec3 velocity = getVelocity();
    
    // Get rotation matrix and convert to quaternion
    RotMat rot_mat = getRotation();
    
    // Convert rotation matrix to quaternion
    Eigen::Quaterniond quat(rot_mat);
    quat.normalize();
    
    // Create and populate odometry message
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = current_time;
    odom_msg->header.frame_id = odom_frame_id_;
    odom_msg->child_frame_id = base_frame_id_;
    
    // Position
    odom_msg->pose.pose.position.x = position(0);
    odom_msg->pose.pose.position.y = position(1);
    odom_msg->pose.pose.position.z = position(2);
    
    // Orientation
    odom_msg->pose.pose.orientation.x = quat.x();
    odom_msg->pose.pose.orientation.y = quat.y();
    odom_msg->pose.pose.orientation.z = quat.z();
    odom_msg->pose.pose.orientation.w = quat.w();
    
    // Velocity (in world frame, which is what odometry typically expects)
    odom_msg->twist.twist.linear.x = velocity(0);
    odom_msg->twist.twist.linear.y = velocity(1);
    odom_msg->twist.twist.linear.z = velocity(2);
    
    // Angular velocity (in base_link frame)
    Vec3 angular_velocity = getGyro();
    odom_msg->twist.twist.angular.x = angular_velocity(0);
    odom_msg->twist.twist.angular.y = angular_velocity(1);
    odom_msg->twist.twist.angular.z = angular_velocity(2);
    
    // Set covariance matrices (you can adjust these values based on your estimation accuracy)
    // Position covariance
    odom_msg->pose.covariance[0] = 0.1;   // x
    odom_msg->pose.covariance[7] = 0.1;   // y  
    odom_msg->pose.covariance[14] = 0.1;  // z
    odom_msg->pose.covariance[21] = 0.05; // roll
    odom_msg->pose.covariance[28] = 0.05; // pitch
    odom_msg->pose.covariance[35] = 0.05; // yaw
    
    // Velocity covariance
    odom_msg->twist.covariance[0] = 0.1;   // vx
    odom_msg->twist.covariance[7] = 0.1;   // vy
    odom_msg->twist.covariance[14] = 0.1;  // vz
    odom_msg->twist.covariance[21] = 0.05; // wx
    odom_msg->twist.covariance[28] = 0.05; // wy
    odom_msg->twist.covariance[35] = 0.05; // wz
    
    // Publish odometry
    odom_publisher_->publish(std::move(odom_msg));
    
    // Create and publish transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = current_time;
    transform_stamped.header.frame_id = odom_frame_id_;
    transform_stamped.child_frame_id = base_frame_id_;
    
    // Position
    transform_stamped.transform.translation.x = position(0);
    transform_stamped.transform.translation.y = position(1);
    transform_stamped.transform.translation.z = position(2);
    
    // Orientation
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();
    
    // Send transform
    tf_broadcaster_->sendTransform(transform_stamped);
}
