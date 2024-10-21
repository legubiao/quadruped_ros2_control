//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_controller/FSM/StateRL.h"

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

StateRL::StateRL(CtrlComponent &ctrl_component, const std::string &config_path,
                 const std::vector<double> &target_pos) : FSMState(
    FSMStateName::RL, "rl", ctrl_component) {
    for (int i = 0; i < 12; i++) {
        init_pos_[i] = target_pos[i];
    }

    // read params from yaml
    loadYaml(config_path);

    // history
    if (!params_.observations_history.empty()) {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.observations_history.size());
    }

    std::cout << "Model loading: " << config_path + "/" + params_.model_name << std::endl;
    model_ = torch::jit::load(config_path + "/" + params_.model_name);



    // for (const auto &param: model_.parameters()) {
    //     std::cout << "Parameter dtype: " << param.dtype() << std::endl;
    // }


    rl_thread_ = std::thread([&] {
        while (true) {
            try {
                executeAndSleep(
                    [&] {
                        if (running_) {
                            runModel();
                        }
                    },
                    ctrl_comp_.frequency_ / params_.decimation);
            } catch (const std::exception &e) {
                running_ = false;
                RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "Error in RL thread: %s", e.what());
            }
        }
    });
}

void StateRL::enter() {
    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});

    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // Init control
    control_.x = 0.0;
    control_.y = 0.0;
    control_.yaw = 0.0;

    running_ = true;
}

void StateRL::run() {
    getState();
    setCommand();
}

void StateRL::exit() {
    running_ = false;
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
            obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
        } else if (observation == "ang_vel") {
            obs_list.push_back(
                quatRotateInverse(obs_.base_quat, obs_.ang_vel, params_.framework) * params_.ang_vel_scale);
        } else if (observation == "gravity_vec") {
            obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
        } else if (observation == "commands") {
            obs_list.push_back(obs_.commands * params_.commands_scale);
        } else if (observation == "dof_pos") {
            obs_list.push_back((obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale);
        } else if (observation == "dof_vel") {
            obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
        } else if (observation == "actions") {
            obs_list.push_back(obs_.actions);
        }
    }

    const torch::Tensor obs = cat(obs_list, 1);

    // std::cout << "Observation: " << obs << std::endl;
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
    if (config["observations_history"].IsNull()) {
        params_.observations_history = {};
    } else {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
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
    params_.torque_limits = torch::tensor(
        ReadVectorFromYaml<double>(config["torque_limits"], params_.framework, rows, cols)).view({1, -1});

    params_.default_dof_pos = torch::from_blob(init_pos_, {12}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);

    // params_.default_dof_pos = torch::tensor(
    //     ReadVectorFromYaml<double>(config["default_dof_pos"], params_.framework, rows, cols)).view({1, -1});
}

torch::Tensor StateRL::quatRotateInverse(const torch::Tensor &q, const torch::Tensor &v, const std::string &framework) {
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim") {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    } else if (framework == "isaacgym") {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateRL::forward() {
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    if (!params_.observations_history.empty()) {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        actions = model_.forward({history_obs_}).toTensor();
    } else {
        actions = model_.forward({clamped_obs}).toTensor();
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0) {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

void StateRL::getState() {
    if (params_.framework == "isaacgym") {
        robot_state_.imu.quaternion[3] = ctrl_comp_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[0] = ctrl_comp_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_comp_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_comp_.imu_state_interface_[3].get().get_value();
    } else if (params_.framework == "isaacsim") {
        robot_state_.imu.quaternion[0] = ctrl_comp_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_comp_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_comp_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[3] = ctrl_comp_.imu_state_interface_[3].get().get_value();
    }

    robot_state_.imu.gyroscope[0] = ctrl_comp_.imu_state_interface_[4].get().get_value();
    robot_state_.imu.gyroscope[1] = ctrl_comp_.imu_state_interface_[5].get().get_value();
    robot_state_.imu.gyroscope[2] = ctrl_comp_.imu_state_interface_[6].get().get_value();

    robot_state_.imu.accelerometer[0] = ctrl_comp_.imu_state_interface_[7].get().get_value();
    robot_state_.imu.accelerometer[1] = ctrl_comp_.imu_state_interface_[8].get().get_value();
    robot_state_.imu.accelerometer[2] = ctrl_comp_.imu_state_interface_[9].get().get_value();

    for (int i = 0; i < 12; i++) {
        robot_state_.motor_state.q[i] = ctrl_comp_.joint_position_state_interface_[i].get().get_value();
        robot_state_.motor_state.dq[i] = ctrl_comp_.joint_velocity_state_interface_[i].get().get_value();
        robot_state_.motor_state.tauEst[i] = ctrl_comp_.joint_effort_state_interface_[i].get().get_value();
    }

    control_.x = ctrl_comp_.control_inputs_.ly;
    control_.y = -ctrl_comp_.control_inputs_.lx;
    control_.yaw = -ctrl_comp_.control_inputs_.rx;

    updated_ = true;
}

void StateRL::runModel() {
    if (ctrl_comp_.enable_estimator_) {
        obs_.lin_vel = torch::from_blob(ctrl_comp_.estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
        to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope).unsqueeze(0);
    obs_.commands = torch::tensor({{control_.x, control_.y, control_.yaw}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    const torch::Tensor clamped_actions = forward();

    for (const int i: params_.hip_scale_reduction_indices) {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

    output_dof_pos_ = actions_scaled + params_.default_dof_pos;

    for (int i = 0; i < params_.num_of_dofs; ++i) {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }
}

void StateRL::setCommand() const {
    for (int i = 0; i < 12; i++) {
        ctrl_comp_.joint_position_command_interface_[i].get().set_value(robot_command_.motor_command.q[i]);
        ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(robot_command_.motor_command.dq[i]);
        ctrl_comp_.joint_kp_command_interface_[i].get().set_value(robot_command_.motor_command.kp[i]);
        ctrl_comp_.joint_kd_command_interface_[i].get().set_value(robot_command_.motor_command.kd[i]);
        ctrl_comp_.joint_torque_command_interface_[i].get().set_value(robot_command_.motor_command.tau[i]);
    }
}
