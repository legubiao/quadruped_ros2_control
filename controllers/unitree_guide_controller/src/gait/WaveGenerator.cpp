//
// Created by biao on 24-9-18.
//

#include "unitree_guide_controller/gait/WaveGenerator.h"

#include <iostream>

WaveGenerator::WaveGenerator(const double period, const double st_ratio, const Vec4 &bias) {

    phase_past_ << 0.5, 0.5, 0.5, 0.5;
    contact_past_.setZero();
    status_past_ = WaveStatus::SWING_ALL;
    status_ = WaveStatus::SWING_ALL;

    period_ = period;
    st_ratio_ = st_ratio;
    bias_ = bias;

    if (st_ratio_ >= 1 || st_ratio_ <= 0) {
        std::cerr << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)"
                << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias_.rows(); i++) {
        if (bias_(i) > 1 || bias_(i) < 0) {
            std::cerr << "[ERROR] The bias of WaveGenerator should between [0, 1]"
                    << std::endl;
            exit(-1);
        }
    }
    start_t_ = getSystemTime();
}

auto WaveGenerator::update() -> void {
    calcWave(phase_, contact_, status_);

    if (status_ != status_past_) {
        if (switch_status_.sum() == 0) {
            switch_status_.setOnes();
        }
        calcWave(phase_past_, contact_past_, status_past_);

        if (status_ == WaveStatus::STANCE_ALL && status_past_ == WaveStatus::SWING_ALL) {
            contact_past_.setOnes();
        } else if (status_ == WaveStatus::SWING_ALL && status_past_ == WaveStatus::STANCE_ALL) {
            contact_past_.setZero();
        }
    }

    if (switch_status_.sum() != 0) {
        for (int i(0); i < 4; ++i) {
            if (contact_(i) == contact_past_(i)) {
                switch_status_(i) = 0;
            } else {
                contact_(i) = contact_past_(i);
                phase_(i) = phase_past_(i);
            }
        }
        if (switch_status_.sum() == 0) {
            status_past_ = status_;
        }
    }
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, const WaveStatus status) {
    switch (status) {
        case WaveStatus::WAVE_ALL: {
            const double past_t = static_cast<double>(getSystemTime() - start_t_) * 1e-6;
            for (int i(0); i < 4; ++i) {
                normal_t_(i) =
                        fmod(past_t + period_ - period_ * bias_(i), period_) / period_;
                if (normal_t_(i) < st_ratio_) {
                    contact(i) = 1;
                    phase(i) = normal_t_(i) / st_ratio_;
                } else {
                    contact(i) = 0;
                    phase(i) = (normal_t_(i) - st_ratio_) / (1 - st_ratio_);
                }
            }
            break;
        }
        case WaveStatus::SWING_ALL: {
            contact.setZero();
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        }
        case WaveStatus::STANCE_ALL: {
            contact.setOnes();
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        }
    }
}
