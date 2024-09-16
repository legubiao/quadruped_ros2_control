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


#endif //MATHTOOLS_H
