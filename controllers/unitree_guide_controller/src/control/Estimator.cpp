//
// Created by biao on 24-9-14.
//

#include "unitree_guide_controller/control/Estimator.h"
#include "unitree_guide_controller/control/CtrlComponent.h"
#include <unitree_guide_controller/common/mathTools.h>

Estimator::Estimator() {
    g_ = KDL::Vector(0, 0, -9.81);
}

void Estimator::run(const CtrlComponent &ctrlComp, std::vector<KDL::Frame> feet_poses_,
                    std::vector<KDL::Vector> feet_vels_,
                    const std::vector<int> &contact, const std::vector<double> &phase) {
    _Q = _QInit;
    _R = _RInit;

    // Adjust the covariance based on foot contact and phase.
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 0) {
            _Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = _largeVariance * Eigen::MatrixXd::Identity(3, 3);
            _R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = _largeVariance * Eigen::MatrixXd::Identity(3, 3);
            _R(24 + i, 24 + i) = _largeVariance;
        } else {
            _trust = windowFunc(phase[i], 0.2);
            _Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) =
                    (1 + (1 - _trust) * _largeVariance) *
                    _QInit.block(6 + 3 * i, 6 + 3 * i, 3, 3);
            _R.block(12 + 3 * i, 12 + 3 * i, 3, 3) =
                    (1 + (1 - _trust) * _largeVariance) *
                    _RInit.block(12 + 3 * i, 12 + 3 * i, 3, 3);
            _R(24 + i, 24 + i) =
                    (1 + (1 - _trust) * _largeVariance) * _RInit(24 + i, 24 + i);
        }
        _feetPos2Body.segment(3 * i, 3) = Eigen::Map<Eigen::Vector3d>(feet_poses_[i].p.data);
        _feetVel2Body.segment(3 * i, 3) = Eigen::Map<Eigen::Vector3d>(feet_vels_[i].data);
    }

    // Acceleration from imu as system input
    const KDL::Rotation rotation = KDL::Rotation::Quaternion(ctrlComp.imu_state_interface_[7].get().get_value(),
                                                             ctrlComp.imu_state_interface_[8].get().get_value(),
                                                             ctrlComp.imu_state_interface_[9].get().get_value(),
                                                             ctrlComp.imu_state_interface_[6].get().get_value());
    const KDL::Vector acc(ctrlComp.imu_state_interface_[3].get().get_value(),
                          ctrlComp.imu_state_interface_[4].get().get_value(),
                          ctrlComp.imu_state_interface_[5].get().get_value());
    _u = Eigen::Map<Eigen::Vector3d>((rotation * acc + g_).data);
    _xhat = _A * _xhat + _B * _u;
    _yhat = _C * _xhat;
}
