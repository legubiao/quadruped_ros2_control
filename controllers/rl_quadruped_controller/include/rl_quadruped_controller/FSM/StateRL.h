//
// Created by biao on 24-10-6.
//

#ifndef STATERL_H
#define STATERL_H

#include <common/ObservationBuffer.h>
#include <rl_quadruped_controller/control/CtrlComponent.h>
#include <torch/script.h>

#include "controller_common/FSM/FSMState.h"

struct CtrlComponent;

template <typename Functor>
void executeAndSleep(Functor f, const double frequency)
{
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();

    // Execute wrapped function
    f();

    // Compute desired duration rounded to clock decimation
    const std::chrono::duration<double> desiredDuration(1.0 / frequency);
    const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

    // Sleep
    const auto sleepTill = start + dt;
    std::this_thread::sleep_until(sleepTill);
}

inline void setThreadPriority(int priority, std::thread& thread)
{
    sched_param sched{};
    sched.sched_priority = priority;

    if (priority != 0)
    {
        if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0)
        {
            std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
                "that the user and the group permissions are not set properly.)"
                << std::endl;
        }
    }
}


template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

struct Control
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
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
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
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

class StateRL final : public FSMState
{
public:
    explicit StateRL(CtrlInterfaces& ctrl_interfaces,
                     CtrlComponent& ctrl_component,
                     const std::vector<double>& target_pos);

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    torch::Tensor computeObservation();

    void loadYaml(const std::string& config_path);

    static torch::Tensor quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v,
                                           const std::string& framework);

    /**
    * @brief Forward the RL model to get the action
    */
    torch::Tensor forward();

    void getState();

    void runModel();

    void setCommand() const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::string robot_pkg_ = "go2_description";
    std::string model_folder_ = "legged_gym";

    bool enable_estimator_;
    std::shared_ptr<Estimator>& estimator_;

    // Parameters
    ModelParams params_;
    Observations obs_;
    Control control_;
    double init_pos_[12] = {};

    RobotState<double> robot_state_;
    RobotCommand<double> robot_command_;

    // history buffer
    std::shared_ptr<ObservationBuffer> history_obs_buf_;
    torch::Tensor history_obs_;

    // rl module
    torch::jit::script::Module model_;
    bool use_rl_thread_ = true;
    std::thread rl_thread_;
    bool running_ = false;
    bool updated_ = false;

    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos_;
};


#endif //STATERL_H
