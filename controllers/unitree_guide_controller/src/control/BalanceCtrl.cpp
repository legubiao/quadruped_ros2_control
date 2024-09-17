//
// Created by tlab-uav on 24-9-16.
//

#include "unitree_guide_controller/control/BalanceCtrl.h"

#include <unitree_guide_controller/common/mathTools.h>
#include <unitree_guide_controller/quadProgpp/QuadProg++.hh>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

BalanceCtrl::BalanceCtrl() {
    _mass = 15;
    _alpha = 0.001;
    _beta = 0.1;
    _fricRatio = 0.4;
    _g << 0, 0, -9.81;
}

void BalanceCtrl::init(const QuadrupedRobot &robot) {
    _mass = robot.mass_;
    _pcb = KDL::Vector(0.0, 0.0, 0.0);
    _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();

    Vec6 s;
    Vec12 w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;

    s << 20, 20, 50, 450, 450, 450;
    _S = s.asDiagonal();
    _W = w.asDiagonal();
    _U = u.asDiagonal();

    _Fprev.setZero();
    _fricMat << 1, 0, _fricRatio, -1, 0, _fricRatio, 0, 1, _fricRatio, 0, -1,
            _fricRatio, 0, 0, 1;
}

std::vector<KDL::Vector> BalanceCtrl::calF(const Vec3 &ddPcd, const Vec3 &dWbd, const KDL::Rotation &rotM,
                                           const std::vector<KDL::Vector> &feetPos2B, const std::vector<int> &contact) {
    calMatrixA(feetPos2B, rotM);
    calVectorBd(ddPcd, dWbd, rotM);
    calConstraints(contact);

    _G = _A.transpose() * _S * _A + _alpha * _W + _beta * _U;
    _g0T = -_bd.transpose() * _S * _A - _beta * _Fprev.transpose() * _U;

    solveQP();
    _Fprev = _F;
    std::vector<KDL::Vector> res;
    res.resize(4);
    for (int i = 0; i < 4; ++i) {
        res[i] = KDL::Vector(_F(i * 3), _F(i * 3 + 1), _F(i * 3 + 2));
    }
    return res;
}

void BalanceCtrl::calMatrixA(const std::vector<KDL::Vector> &feetPos2B, const KDL::Rotation &rotM) {
    for (int i = 0; i < 4; ++i) {
        _A.block(0, 3 * i, 3, 3) = I3;
        KDL::Vector tempVec = feetPos2B[i] - rotM * _pcb;
        _A.block(3, 3 * i, 3, 3) = skew(tempVec);
    }
}

void BalanceCtrl::calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const KDL::Rotation &rotM) {
    _bd.head(3) = _mass * (ddPcd - _g);
    _bd.tail(3) = Eigen::Matrix3d(rotM.data) * _Ib * Eigen::Matrix3d(rotM.data).transpose() *
                  dWbd;
}

void BalanceCtrl::calConstraints(const std::vector<int> &contact) {
    int contactLegNum = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            contactLegNum += 1;
        }
    }
    _CI.resize(5 * contactLegNum, 12);
    _ci0.resize(5 * contactLegNum);
    _CE.resize(3 * (4 - contactLegNum), 12);
    _ce0.resize(3 * (4 - contactLegNum));

    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;
    int ciID = 0;
    for (int i(0); i < 4; ++i) {
        if (contact[i] == 1) {
            _CI.block(5 * ciID, 3 * i, 5, 3) = _fricMat;
            ++ciID;
        } else {
            _CE.block(3 * ceID, 3 * i, 3, 3) = I3;
            ++ceID;
        }
    }
}

void BalanceCtrl::solveQP() {
    const long n = _F.size();
    const long m = _ce0.size();
    const long p = _ci0.size();

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}
