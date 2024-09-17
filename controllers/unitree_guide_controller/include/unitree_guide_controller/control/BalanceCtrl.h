//
// Created by tlab-uav on 24-9-16.
//

#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <kdl/frames.hpp>

#include "unitree_guide_controller/common/mathTypes.h"
class QuadrupedRobot;

class BalanceCtrl {
public:
    explicit BalanceCtrl();

    ~BalanceCtrl() = default;

    void init(const QuadrupedRobot &robot);

    Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM,
               const std::vector<KDL::Vector> &feetPos2B, const std::vector<int> &contact);

private:
    void calMatrixA(const std::vector<KDL::Vector> &feetPos2B, const RotMat &rotM);

    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM);

    void calConstraints(const std::vector<int> &contact);

    void solveQP();

    Mat12 G_, W_, U_;
    Mat6 S_;
    Mat3 Ib_;
    Vec6 bd_;
    Vec3 _g, _pcb;
    Vec12 F_, F_prev_, g0T_;
    double mass_, alpha_, beta_, friction_ratio_;
    Eigen::MatrixXd CE_, CI_;
    Eigen::VectorXd ce0_, ci0_;
    Eigen::Matrix<double, 6, 12> A_;
    Eigen::Matrix<double, 5, 3> friction_mat_;
};


#endif //BALANCECTRL_H
