//
// Created by tlab-uav on 24-9-12.
//

#ifndef MATHTOOLS_H
#define MATHTOOLS_H

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max,
                const double minLim = -1, const double maxLim = 1) {
    return (value - minLim) * (max - min) / (maxLim - minLim) + min;
}

#endif //MATHTOOLS_H
