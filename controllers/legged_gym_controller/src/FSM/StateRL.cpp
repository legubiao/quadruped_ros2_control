//
// Created by biao on 24-10-6.
//

#include "legged_gym_controller/FSM/StateRL.h"

StateRL::StateRL(CtrlComponent &ctrl_component) : FSMState(
    FSMStateName::RL, "rl", ctrl_component) {
}

void StateRL::enter() {
}

void StateRL::run() {
}

void StateRL::exit() {
}

FSMStateName StateRL::checkChange() {
    switch (ctrl_comp_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDDOWN;
        default:
            return FSMStateName::RL;
    }
}

torch::Tensor StateRL::computeObservation() {
    std::vector<torch::Tensor> obs_list;

    const torch::Tensor obs = cat(obs_list, 1);
    torch::Tensor clamped_obs = clamp(obs, -clip_obs_, clip_obs_);
    return clamped_obs;
}

torch::Tensor StateRL::forward() {
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    if (use_history_) {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec({0, 1, 2, 3, 4, 5});
        actions = model.forward({history_obs_}).toTensor();
    } else {
        actions = model.forward({clamped_obs}).toTensor();
    }

    if (clip_actions_upper_.numel() != 0 && clip_actions_lower_.numel() != 0) {
        return clamp(actions, clip_actions_lower_, clip_actions_upper_);
    }
    return actions;
}
