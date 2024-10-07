//
// Created by biao on 24-10-6.
//

#ifndef STATERL_H
#define STATERL_H

#include <common/ObservationBuffer.h>
#include <torch/script.h>

#include "FSMState.h"

template<typename T>
struct RobotCommand {
    struct MotorCommand {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template<typename T>
struct RobotState {
    struct IMU {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

struct ModelParams {
    std::string model_name;
    std::string framework;
    bool use_history;
    double dt;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    double damping;
    double stiffness;
    double action_scale;
    double hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    torch::Tensor clip_actions_upper;
    torch::Tensor clip_actions_lower;
    torch::Tensor torque_limits;
    torch::Tensor rl_kd;
    torch::Tensor rl_kp;
    torch::Tensor fixed_kp;
    torch::Tensor fixed_kd;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    std::vector<std::string> joint_controller_names;
};

struct Observations
{
    torch::Tensor lin_vel;
    torch::Tensor ang_vel;
    torch::Tensor gravity_vec;
    torch::Tensor commands;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor actions;
};

class StateRL final : public FSMState {
public:
    explicit StateRL(CtrlComponent &ctrl_component, const std::string &config_path);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    torch::Tensor computeObservation();

    void loadYaml(const std::string &config_path);

    static torch::Tensor quatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework);

    /**
    * @brief Forward the RL model to get the action
    */
    torch::Tensor forward();

    // Parameters
    ModelParams params_;
    Observations obs;

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
