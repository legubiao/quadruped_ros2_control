//
// Created by biao on 24-9-18.
//
#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <controller_common/common/enumClass.h>
#include <basic_quadruped_controller/common/mathTypes.h>

// Forward declaration
enum class FSMStateName;

class WaveGenerator {
public:
    WaveGenerator(double period, double st_ratio, const Vec4 &bias, const rclcpp::Time& start_time);

    ~WaveGenerator() = default;

    void update(const rclcpp::Time& time);

    /**
     * Set wave generator status
     * @param status new wave status
     */
    void setStatus(WaveStatus status) { status_ = status; }

    /**
     * Set wave generator status based on FSM state
     * @param fsm_state FSM state name
     */
    void setStatusFromFSM(FSMStateName fsm_state);

    /**
     * Get current wave status
     * @return current wave status
     */
    [[nodiscard]] WaveStatus getStatus() const { return status_; }

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double get_t() const { return period_; }

    Vec4 phase_;
    VecInt4 contact_;
    WaveStatus status_{};

private:
    /**
     * Update phase, contact and status based on current time.
     * @param phase foot phase
     * @param contact foot contact
     * @param status Wave Status
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double period_{};
    double st_ratio_{}; // stance phase ratio
    Vec4 bias_;

    Vec4 normal_t_; // normalize time [0,1)
    Vec4 phase_past_; // foot phase
    VecInt4 contact_past_; // foot contact
    VecInt4 switch_status_;
    WaveStatus status_past_;

    rclcpp::Time start_time_;  // 启动时间
    double accumulated_time_{0.0};  // 累积时间
};
