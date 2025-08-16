//
// Created by tlab-uav on 25-2-27.
//

// #include <basic_quadruped_controller/gait/WaveGenerator.h>
#pragma once
#include "QuadrupedKinematic.h"
//
// #include "BalanceCtrl.h"
// #include "Estimator.h"

struct CtrlComponent {
    std::shared_ptr<QuadrupedKinematic> robot_model_;
    // std::shared_ptr<Estimator> estimator_;
    // std::shared_ptr<BalanceCtrl> balance_ctrl_;
    // std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};
