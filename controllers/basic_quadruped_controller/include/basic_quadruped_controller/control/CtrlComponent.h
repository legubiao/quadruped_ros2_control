//
// Created by tlab-uav on 25-2-27.
//
#pragma once


#include <basic_quadruped_controller/gait/WaveGenerator.h>

#include "BalanceCtrl.h"
#include "Estimator.h"
#include "QuadrupedKinematic.h"

struct CtrlComponent
{
    std::shared_ptr<QuadrupedKinematic> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<BalanceCtrl> balance_ctrl_;
    std::shared_ptr<WaveGenerator> wave_generator_;

    CtrlComponent() = default;
};
