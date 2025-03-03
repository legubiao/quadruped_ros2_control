//
// Created by biao on 24-9-10.
//

#ifndef CtrlComponent_H
#define CtrlComponent_H

#include "Estimator.h"

struct CtrlComponent {

    bool enable_estimator_ = false;
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;

    CtrlComponent() = default;
};

#endif //CtrlComponent_H
