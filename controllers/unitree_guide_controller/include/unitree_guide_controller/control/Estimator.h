//
// Created by biao on 24-9-14.
//

#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <eigen3/Eigen/Dense>
#include <kdl/frames.hpp>

struct CtrlComponent;

class Estimator {
public:
    Estimator();

    ~Estimator() = default;

    /**
     * Get the estimated robot central position
     * @return robot central position
     */
    Eigen::Matrix<double, 3, 1> getPosition() {
        return _xhat.segment(0, 3);
    }

    /**
     * Get the estimated robot central velocity
     * @return robot central velocity
     */
    Eigen::Matrix<double, 3, 1> getVelocity() {
        return _xhat.segment(3, 3);
    }

    void run(
        const CtrlComponent &ctrlComp,
        std::vector<KDL::Frame> feet_poses_,
        std::vector<KDL::Vector> feet_vels_,
        const std::vector<int> &contact,
        const std::vector<double> &phase
    );

private:
    void init();

    Eigen::Matrix<double, 18, 1> _xhat; // The state of estimator, position(3)+velocity(3)+feet position(3x4)

    Eigen::Matrix<double, 3, 1> _u; // The input of estimator

    Eigen::Matrix<double, 28, 1> _y; // The measurement value of output y
    Eigen::Matrix<double, 28, 1> _yhat; // The prediction of output y
    Eigen::Matrix<double, 18, 18> _A; // The transtion matrix of estimator
    Eigen::Matrix<double, 18, 3> _B; // The input matrix
    Eigen::Matrix<double, 28, 18> _C; // The output matrix
    // Covariance Matrix
    Eigen::Matrix<double, 18, 18> _P; // Prediction covariance
    Eigen::Matrix<double, 18, 18> _Ppriori; // Priori prediction covariance
    Eigen::Matrix<double, 18, 18> _Q; // Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28> _R; // Measurement covariance
    Eigen::Matrix<double, 18, 18>
    _QInit; // Initial value of Dynamic simulation covariance
    Eigen::Matrix<double, 28, 28>
    _RInit; // Initial value of Measurement covariance
    Eigen::Matrix<double, 18, 1> _Qdig; // adjustable process noise covariance
    Eigen::Matrix<double, 3, 3> _Cu; // The covariance of system input u
    // Output Measurement
    Eigen::Matrix<double, 12, 1>
    _feetPos2Body; // The feet positions to body, in the global coordinate
    Eigen::Matrix<double, 12, 1>
    _feetVel2Body; // The feet velocity to body, in the global coordinate
    Eigen::Matrix<double, 4, 1>
    _feetH; // The Height of each foot, in the global coordinate
    Eigen::Matrix<double, 28, 28> _S; // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28> > _Slu; // _S.lu()
    Eigen::Matrix<double, 28, 1> _Sy; // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> _Sc; // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> _SR; // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> _STC; // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> _IKC; // _IKC = I - KC

    KDL::Vector g_;

    std::vector<KDL::Frame> feet_poses_;
    std::vector<KDL::Vector> feet_vels_;

    double _trust;
    double _largeVariance;
};


#endif //ESTIMATOR_H
