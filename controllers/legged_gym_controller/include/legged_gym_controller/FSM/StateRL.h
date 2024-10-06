//
// Created by biao on 24-10-6.
//

#ifndef STATERL_H
#define STATERL_H

#include <common/ObservationBuffer.h>
#include <torch/script.h>

#include "FSMState.h"


class StateRL final : public FSMState {
public:
    explicit StateRL(CtrlComponent &ctrl_component);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    torch::Tensor computeObservation();

    /**
    * @brief Forward the RL model to get the action
    */
    torch::Tensor forward();

    // Parameters
    double linear_vel_scale_;
    double angular_vel_scale_;
    double clip_obs_;
    torch::Tensor clip_actions_upper_;
    torch::Tensor clip_actions_lower_;
    bool use_history_;

    // history buffer
    std::shared_ptr<ObservationBuffer> history_obs_buf_;
    torch::Tensor history_obs_;

    // rl module
    torch::jit::script::Module model;
    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos;
};


#endif //STATERL_H
