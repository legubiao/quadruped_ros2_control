//
// Created by biao on 24-9-14.
//

#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <memory>
#include <eigen3/Eigen/Dense>
#include <kdl/frames.hpp>

#include "LowPassFilter.h"

struct CtrlComponent;

class Estimator {
public:
    Estimator();

    ~Estimator() = default;

    /**
     * Get the estimated robot central position
     * @return robot central position
     */
    KDL::Vector getPosition() {
        return {_xhat(0), _xhat(1), _xhat(2)};
    }

    /**
     * Get the estimated robot central velocity
     * @return robot central velocity
     */
    KDL::Vector getVelocity() {
        return {_xhat(3), _xhat(4), _xhat(5)};
    }

    /**
     * Get the estimated foot position in world frame
     * @param index leg index
     * @return foot position in world frame
     */
    KDL::Vector getFootPos(const int index) {
        return getPosition() + rotation_ * foot_poses_[index].p;
    }

    /**
     * Get all estimated foot positions in world frame
     * @return all foot positions in world frame
     */
    std::vector<KDL::Vector> getFootPos() {
        std::vector<KDL::Vector> foot_pos;
        foot_pos.resize(4);
        for (int i = 0; i < 4; i++) {
            foot_pos[i] = getFootPos(i);
        }
        return foot_pos;
    }

    /**
     * Get the estimated foot position in body frame
     * @return
     */
    std::vector<KDL::Vector> getFootPos2Body() {
        std::vector<KDL::Vector> foot_pos;
        foot_pos.resize(4);
        const KDL::Vector body_pos = getPosition();
        for (int i = 0; i < 4; i++) {
            foot_pos[i] = getFootPos(i) - body_pos;
        }
        return foot_pos;
    }

    KDL::Rotation getRotation() {
        return rotation_;
    }

    KDL::Vector getGyro() {
        return gyro_;
    }

    [[nodiscard]] KDL::Vector getGlobalGyro() const {
        return rotation_ * gyro_;
    }

    void update(const CtrlComponent &ctrlComp);

private:
    Eigen::Matrix<double, 18, 1> _xhat; // The state of estimator, position(3)+velocity(3)+feet position(3x4)

    Eigen::Matrix<double, 3, 1> _u; // The input of estimator

    Eigen::Matrix<double, 28, 1> _y; // The measurement value of output y
    Eigen::Matrix<double, 28, 1> _yhat; // The prediction of output y
    Eigen::Matrix<double, 18, 18> A; // The transtion matrix of estimator
    Eigen::Matrix<double, 18, 3> B; // The input matrix
    Eigen::Matrix<double, 28, 18> C; // The output matrix
    // Covariance Matrix
    Eigen::Matrix<double, 18, 18> P; // Prediction covariance
    Eigen::Matrix<double, 18, 18> Ppriori; // Priori prediction covariance
    Eigen::Matrix<double, 18, 18> Q; // Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28> R; // Measurement covariance
    Eigen::Matrix<double, 18, 18>
    QInit; // Initial value of Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28>
    RInit; // Initial value of Measurement covariance
    Eigen::Matrix<double, 18, 1> Qdig; // adjustable process noise covariance
    Eigen::Matrix<double, 3, 3> Cu; // The covariance of system input u
    // Output Measurement
    Eigen::Matrix<double, 12, 1>
    _feetPos2Body; // The feet positions to body, in the global coordinate
    Eigen::Matrix<double, 12, 1>
    _feetVel2Body; // The feet velocity to body, in the global coordinate
    Eigen::Matrix<double, 4, 1>
    _feetH; // The Height of each foot, in the global coordinate
    Eigen::Matrix<double, 28, 28> S; // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28> > Slu; // _S.lu()
    Eigen::Matrix<double, 28, 1> Sy; // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> Sc; // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> SR; // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> STC; // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> IKC; // _IKC = I - KC

    KDL::Vector g_;
    double _dt;

    KDL::Rotation rotation_;
    KDL::Vector acceleration_;
    KDL::Vector gyro_;

    std::vector<KDL::Frame> foot_poses_;
    std::vector<KDL::Vector> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter> > low_pass_filters_;

    double _largeVariance;
};


#endif //ESTIMATOR_H
