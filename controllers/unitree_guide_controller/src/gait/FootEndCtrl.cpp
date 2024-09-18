//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/FootEndCtrl.h"

FootEndCtrl::FootEndCtrl() {
    k_x_ = 0.005;
    k_y_ = 0.005;
    k_yaw_ = 0.005;
}

Vec3 FootEndCtrl::calcFootPos(int index, Vec2 vxy_goal_global, double d_yaw_global, double phase) {
}
