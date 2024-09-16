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

    std::vector<KDL::Vector> calF(const Vec3 &ddPcd, const Vec3 &dWbd, const KDL::Rotation &rotM,
                                  const std::vector<KDL::Vector> &feetPos2B, const std::vector<int> &contact);

private:
    void calMatrixA(const std::vector<KDL::Vector> &feetPos2B, const KDL::Rotation &rotM);

    void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const KDL::Rotation &rotM);

    void calConstraints(const std::vector<int> &contact);

    void solveQP();

    Mat12 _G, _W, _U;
    Mat6 _S;
    Mat3 _Ib;
    Vec6 _bd;
    KDL::Vector _pcb;
    Vec3 _g;
    Vec12 _F, _Fprev, _g0T;
    double _mass, _alpha, _beta, _fricRatio;
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;
    Eigen::Matrix<double, 6, 12> _A;
    Eigen::Matrix<double, 5, 3> _fricMat;
};


#endif //BALANCECTRL_H
