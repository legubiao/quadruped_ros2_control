//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>

#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2::legged_robot {
    KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchio_interface,
                                               CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics &ee_kinematics,
                                               CtrlInterfaces &ctrl_component,
                                               const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimateBase(std::move(info), ctrl_component,
                            node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          ee_kinematics_(ee_kinematics.clone()),
          numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
          dimContacts_(3 * numContacts_),
          numState_(6 + dimContacts_),
          numObserve_(2 * dimContacts_ + numContacts_) {
        xHat_.setZero(numState_);
        ps_.setZero(dimContacts_);
        vs_.setZero(dimContacts_);
        a_.setIdentity(numState_, numState_);
        b_.setZero(numState_, 3);

        matrix_t c1(3, 6), c2(3, 6);
        c1 << matrix3_t::Identity(), matrix3_t::Zero();
        c2 << matrix3_t::Zero(), matrix3_t::Identity();
        c_.setZero(numObserve_, numState_);
        for (ssize_t i = 0; i < numContacts_; ++i) {
            c_.block(3 * i, 0, 3, 6) = c1;
            c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;
            c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;
        }
        c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

        q_.setIdentity(numState_, numState_);
        p_ = 100. * q_;
        r_.setIdentity(numObserve_, numObserve_);
        feet_heights_.setZero(numContacts_);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);
        initPublishers();
    }

    vector_t KalmanFilterEstimate::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        updateJointStates();
        updateContact();
        updateImu();

        scalar_t dt = period.seconds();
        a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
        b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
        b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
        q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
        q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
        q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum;

        vector_t qPino(info_.generalizedCoordinatesNum);
        vector_t vPino(info_.generalizedCoordinatesNum);
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>(); // Only set orientation, let position in origin.
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);

        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));
        // Only set angular velocity, let linear velocity be zero
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

        forwardKinematics(model, data, qPino, vPino);
        updateFramePlacements(model, data);

        const auto eePos = ee_kinematics_->getPosition(vector_t());
        const auto eeVel = ee_kinematics_->getVelocity(vector_t(), vector_t());

        matrix_t q = matrix_t::Identity(numState_, numState_);
        q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position_;
        q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity_;
        q.block(6, 6, dimContacts_, dimContacts_) =
                q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;

        matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
        r.block(0, 0, dimContacts_, dimContacts_) =
                r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
        r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
                r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
        r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
                r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

        for (int i = 0; i < numContacts_; i++) {
            int i1 = 3 * i;

            int qIndex = 6 + i1;
            int rIndex1 = i1;
            int rIndex2 = dimContacts_ + i1;
            int rIndex3 = 2 * dimContacts_ + i;
            bool isContact = contact_flag_[i];

            scalar_t high_suspect_number(100);
            q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
            r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
            r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
            r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

            ps_.segment(3 * i, 3) = -eePos[i];
            ps_.segment(3 * i, 3)[2] += foot_radius_;
            vs_.segment(3 * i, 3) = -eeVel[i];
        }

        vector3_t g(0, 0, -9.81);
        vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_accel_local_ + g;

        vector_t y(numObserve_);
        y << ps_, vs_, feet_heights_;
        xHat_ = a_ * xHat_ + b_ * accel;
        matrix_t at = a_.transpose();
        matrix_t pm = a_ * p_ * at + q;
        matrix_t cT = c_.transpose();
        matrix_t yModel = c_ * xHat_;
        matrix_t ey = y - yModel;
        matrix_t s = c_ * pm * cT + r;

        vector_t sEy = s.lu().solve(ey);
        xHat_ += pm * cT * sEy;

        matrix_t sC = s.lu().solve(c_);
        p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

        matrix_t pt = p_.transpose();
        p_ = (p_ + pt) / 2.0;

        //  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
        //    p_.block(0, 2, 2, 16).setZero();
        //    p_.block(2, 0, 16, 2).setZero();
        //    p_.block(0, 0, 2, 2) /= 10.;
        //  }

        updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        return rbd_state_;
    }

    nav_msgs::msg::Odometry KalmanFilterEstimate::getOdomMsg() {
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
        odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
        odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                odom.pose.covariance[i * 6 + j] = p_(i, j);
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
            }
        }
        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
        vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
        odom.twist.twist.linear.x = twist.x();
        odom.twist.twist.linear.y = twist.y();
        odom.twist.twist.linear.z = twist.z();
        odom.twist.twist.angular.x = angular_vel_local_.x();
        odom.twist.twist.angular.y = angular_vel_local_.y();
        odom.twist.twist.angular.z = angular_vel_local_.z();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
                odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
            }
        }
        return odom;
    }

    void KalmanFilterEstimate::loadSettings(const std::string &task_file, const bool verbose) {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "kalmanFilter.";
        if (verbose) {
            std::cerr << "\n #### Kalman Filter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }

        loadData::loadPtreeValue(pt, foot_radius_, prefix + "footRadius", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_position_, prefix + "imuProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_velocity_, prefix + "imuProcessNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
    }
} // namespace legged
