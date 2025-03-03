//
// Created by tlab-uav on 24-9-12.
//

#ifndef MATHTOOLS_H
#define MATHTOOLS_H
#include <iostream>

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max,
                const double minLim = -1, const double maxLim = 1) {
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

template<typename T>
T windowFunc(const T x, const T windowRatio, const T xRange = 1.0,
             const T yRange = 1.0) {
    if (x < 0 || x > xRange) {
        std::cout << "[ERROR][windowFunc] The x=" << x
                << ", which should between [0, xRange]" << std::endl;
    }
    if (windowRatio <= 0 || windowRatio >= 0.5) {
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio
                << ", which should between [0, 0.5]" << std::endl;
    }

    if (x / xRange < windowRatio) {
        return x * yRange / (xRange * windowRatio);
    }
    if (x / xRange > 1 - windowRatio) {
        return yRange * (xRange - x) / (xRange * windowRatio);
    }
    return yRange;
}

inline Mat3 skew(const Vec3 &v) {
    Mat3 m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return m;
}

inline Vec3 rotMatToExp(const RotMat &rm) {
    double cosValue = rm.trace() / 2.0 - 1 / 2.0;
    if (cosValue > 1.0f) {
        cosValue = 1.0f;
    } else if (cosValue < -1.0f) {
        cosValue = -1.0f;
    }

    const double angle = acos(cosValue);
    Vec3 exp;
    if (fabs(angle) < 1e-5) {
        exp = Vec3(0, 0, 0);
    } else if (fabs(angle - M_PI) < 1e-5) {
        exp = angle * Vec3(rm(0, 0) + 1, rm(0, 1), rm(0, 2)) /
              sqrt(2 * (1 + rm(0, 0)));
    } else {
        exp = angle / (2.0f * sin(angle)) *
              Vec3(rm(2, 1) - rm(1, 2), rm(0, 2) - rm(2, 0), rm(1, 0) - rm(0, 1));
    }
    return exp;
}

template<typename T>
T saturation(const T a, Vec2 limits) {
    T lowLim, highLim;
    if (limits(0) > limits(1)) {
        lowLim = limits(1);
        highLim = limits(0);
    } else {
        lowLim = limits(0);
        highLim = limits(1);
    }

    if (a < lowLim) {
        return lowLim;
    }
    if (a > highLim) {
        return highLim;
    }
    return a;
}

inline RotMat quatToRotMat(const Quat &q) {
    double e0 = q(0);
    double e1 = q(1);
    double e2 = q(2);
    double e3 = q(3);

    RotMat R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return R;
}

inline Vec3 rotMatToRPY(const Mat3 &R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = asin(-R(2, 0));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}

inline RotMat rotz(const double &theta) {
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    RotMat R;
    R << c, -s, 0, s, c, 0, 0, 0, 1;
    return R;
}

#endif //MATHTOOLS_H
