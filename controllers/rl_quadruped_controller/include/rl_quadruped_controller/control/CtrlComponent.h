//
// Created by biao on 24-9-10.
//

#ifndef CtrlComponent_H
#define CtrlComponent_H

#include "Estimator.h"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

struct CtrlComponent {

    bool enable_estimator_ = false;
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

    CtrlComponent() = default;
};

#endif //CtrlComponent_H
