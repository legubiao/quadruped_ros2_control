//
// Created by biao on 24-10-6.
//

#include "legged_gym_controller/FSM/StateRL.h"

#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node) {
    std::vector<T> values;
    for (const auto &val: node) {
        values.push_back(val.as<T>());
    }
    return values;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node, const std::string &framework, const int &rows,
                                  const int &cols) {
    std::vector<T> values;
    for (const auto &val: node) {
        values.push_back(val.as<T>());
    }

    if (framework == "isaacsim") {
        std::vector<T> transposed_values(cols * rows);
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                transposed_values[c * rows + r] = values[r * cols + c];
            }
        }
        return transposed_values;
    }
    if (framework == "isaacgym") {
        return values;
    }
    throw std::invalid_argument("Unsupported framework: " + framework);
}

StateRL::StateRL(CtrlComponent &ctrl_component, const std::string &config_path) : FSMState(
    FSMStateName::RL, "rl", ctrl_component) {
    // read params from yaml
    loadYaml(config_path);

    // history
    if (params_.use_history) {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations, 6);
    }

    model = torch::jit::load(config_path + "/" + params_.model_name);

    std::cout << "Model loaded: " << config_path + "/" + params_.model_name << std::endl;
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

    for (const std::string &observation: params_.observations) {
        if (observation == "lin_vel") {
            obs_list.push_back(obs.lin_vel * params_.lin_vel_scale);
        } else if (observation == "ang_vel") {
            // obs_list.push_back(obs.ang_vel * params_.ang_vel_scale); // TODO is QuatRotateInverse necessery?
            obs_list.push_back(
                this->QuatRotateInverse(obs.base_quat, obs.ang_vel, params_.framework) * params_.ang_vel_scale);
        } else if (observation == "gravity_vec") {
            obs_list.push_back(this->QuatRotateInverse(obs.base_quat, obs.gravity_vec, params_.framework));
        } else if (observation == "commands") {
            obs_list.push_back(obs.commands * params_.commands_scale);
        } else if (observation == "dof_pos") {
            obs_list.push_back((obs.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale);
        } else if (observation == "dof_vel") {
            obs_list.push_back(obs.dof_vel * params_.dof_vel_scale);
        } else if (observation == "actions") {
            obs_list.push_back(obs.actions);
        }
    }

    torch::Tensor obs = cat(obs_list, 1);
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

void StateRL::loadYaml(const std::string &config_path) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path + "/config.yaml");
    } catch ([[maybe_unused]] YAML::BadFile &e) {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        return;
    }

    params_.model_name = config["model_name"].as<std::string>();

    params_.model_name = config["model_name"].as<std::string>();
    params_.framework = config["framework"].as<std::string>();
    const int rows = config["rows"].as<int>();
    const int cols = config["cols"].as<int>();
    params_.use_history = config["use_history"].as<bool>();
    params_.dt = config["dt"].as<double>();
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull()) {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    } else {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"], params_.framework, rows, cols)).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"], params_.framework, rows, cols)).view({1, -1});
    }
    params_.action_scale = config["action_scale"].as<double>();
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    // params_.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    params_.commands_scale = torch::tensor({params_.lin_vel_scale, params_.lin_vel_scale, params_.ang_vel_scale});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"], params_.framework, rows, cols)).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"], params_.framework, rows, cols)).view({
        1, -1
    });
    params_.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"], params_.framework, rows, cols)).
            view({1, -1});
    params_.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"], params_.framework, rows, cols)).
            view({1, -1});
    params_.torque_limits = torch::tensor(
        ReadVectorFromYaml<double>(config["torque_limits"], params_.framework, rows, cols)).view({1, -1});
    params_.default_dof_pos = torch::tensor(
        ReadVectorFromYaml<double>(config["default_dof_pos"], params_.framework, rows, cols)).view({1, -1});
    params_.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"],
                                                                     params_.framework, rows, cols);
}

torch::Tensor StateRL::quatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string &framework) {
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim") {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    } else if (framework == "isaacgym") {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateRL::forward() {
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    if (params_.use_history) {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec({0, 1, 2, 3, 4, 5});
        actions = model.forward({history_obs_}).toTensor();
    } else {
        actions = model.forward({clamped_obs}).toTensor();
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0) {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}
