//
// Created by biao on 24-9-18.
//


#ifndef FOOTENDCTRL_H
#define FOOTENDCTRL_H
#include <unitree_guide_controller/common/mathTypes.h>


class FootEndCtrl {
public:
    FootEndCtrl();

    ~FootEndCtrl() = default;

    Vec3 calcFootPos(int index, Vec2 vxy_goal_global, double d_yaw_global, double phase);

private:
    double k_x_, k_y_, k_yaw_;
};


#endif //FOOTENDCTRL_H
