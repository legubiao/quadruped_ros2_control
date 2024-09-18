//
// Created by biao on 24-9-18.
//


#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H
#include <sys/time.h>
#include <unitree_guide_controller/common/enumClass.h>
#include <unitree_guide_controller/common/mathTypes.h>

inline long long getSystemTime() {
    timeval t{};
    gettimeofday(&t, nullptr);
    return 1000000 * t.tv_sec + t.tv_usec;
}

class WaveGenerator {
public:
    WaveGenerator();

    ~WaveGenerator() = default;

    void init(double period, double st_ratio, const Vec4 &bias);

    void calculate(Vec4 &phase_result, VecInt4 &contact_result, WaveStatus status);

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double get_t() const { return period_; }

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
    Vec4 phase_, phase_past_; // foot phase
    VecInt4 contact_, contact_past_; // foot contact
    VecInt4 switch_status_;
    WaveStatus status_past_;

    long start_t_{};
};


#endif //WAVEGENERATOR_H
