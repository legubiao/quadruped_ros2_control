//
// Created by qiayuan on 2021/11/15.
//

#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <memory>

namespace ocs2::legged_robot {
    using namespace legged_robot;

    StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                         const PinocchioEndEffectorKinematics &eeKinematics,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()),
          rbdState_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node)) {
        odomPub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        posePub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
    }

    void StateEstimateBase::updateJointStates(const vector_t &jointPos, const vector_t &jointVel) {
        rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
        rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
    }

    void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t> &quat, const vector3_t &angularVelLocal,
                                      const vector3_t &linearAccelLocal, const matrix3_t &orientationCovariance,
                                      const matrix3_t &angularVelCovariance, const matrix3_t &linearAccelCovariance) {
        quat_ = quat;
        angularVelLocal_ = angularVelLocal;
        linearAccelLocal_ = linearAccelLocal;
        orientationCovariance_ = orientationCovariance;
        angularVelCovariance_ = angularVelCovariance;
        linearAccelCovariance_ = linearAccelCovariance;

        vector3_t zyx = quatToZyx(quat) - zyxOffset_;
        vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::updateAngular(const vector3_t &zyx, const vector_t &angularVel) {
        rbdState_.segment<3>(0) = zyx;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel) {
        rbdState_.segment<3>(3) = pos;
        rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) {
        rclcpp::Time time = odom.header.stamp;
        scalar_t publishRate = 200;
        // if (lastPub_ + rclcpp::Duration(1. / publishRate) < time) {
        //     lastPub_ = time;
        //     if (odomPub_->trylock()) {
        //         odomPub_->msg_ = odom;
        //         odomPub_->unlockAndPublish();
        //     }
        //     if (posePub_->trylock()) {
        //         posePub_->msg_.header = odom.header;
        //         posePub_->msg_.pose = odom.pose;
        //         posePub_->unlockAndPublish();
        //     }
        // }
    }
} // namespace legged
