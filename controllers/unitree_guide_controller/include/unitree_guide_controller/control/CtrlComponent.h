//
// Created by tlab-uav on 25-2-27.
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H
#include <unitree_guide_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"

struct CtrlComponent {
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};
#endif //CTRLCOMPONENT_H
