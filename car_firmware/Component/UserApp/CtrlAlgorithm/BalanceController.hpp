#pragma once

#include "PID.hpp"
#include <algorithm>
#include <cmath>

namespace balance_control {

// Trial settings live in RAM. Flash PID gains retain their units and format.
// Gain scheduling is an experiment, not a stability proof.
struct Options {
    bool gyro_damping{true};
    bool convergence{true};
    float near_kp_ratio{0.85F};
    float angle_window_degrees{2.0F};
    float rate_window_dps{20.0F};
    float rate_filter_hz{0.0F}; // Bypass: do not add unmeasured phase lag by default.
};

// ZYX convention of MPU6050::QuatToEuler. Body Y alone equals pitch rate only
// at zero roll. Inputs must already have the gyro bias removed, in rad/s.
inline float pitchRateDegrees(float roll_degrees, double gyro_y, double gyro_z)
{
    constexpr double radians_per_degree = 0.017453292519943295;
    const double roll = roll_degrees * radians_per_degree;
    return static_cast<float>((std::cos(roll) * gyro_y - std::sin(roll) * gyro_z) /
                              radians_per_degree);
}

inline bool allowIntegration(const Options& options, float error, float rate_dps)
{
    return !options.convergence ||
        (std::abs(error) <= options.angle_window_degrees &&
         std::abs(rate_dps) <= options.rate_window_dps);
}

inline float proportionalScale(const Options& options, float error, float rate_dps)
{
    if (!options.convergence) return 1.0F;
    // Soften P only when both angle and rate are small. Retain all D damping.
    const float distance = std::clamp(std::max(
        std::abs(error) / options.angle_window_degrees,
        std::abs(rate_dps) / options.rate_window_dps), 0.0F, 1.0F);
    const float smooth = distance * distance * (3.0F - 2.0F * distance);
    return options.near_kp_ratio + (1.0F - options.near_kp_ratio) * smooth;
}

struct Output {
    float pwm{};
    float effective_kp{};
    float filtered_pitch_rate_dps{};
    PID::Terms terms{};
};

class AngleController {
public:
    // Stored Kd was tuned against angle differences at 100 Hz. This conversion
    // is fixed, NOT the elapsed interval: jitter must not scale physical damping.
    static constexpr float gain_reference_period_seconds = 0.01F;

    Output update(float target, float measured, float pitch_rate_dps, float elapsed_seconds,
                  float kp, float ki, float kd, const Options& options,
                  float available_pwm = 1000.0F)
    {
        if (!rate_initialized_ || options.rate_filter_hz != previous_filter_hz_ ||
            options.rate_filter_hz <= 0.0F) {
            filtered_rate_ = pitch_rate_dps;
        } else if (elapsed_seconds > 0.0F) {
            constexpr float two_pi = 6.28318530718F;
            const float alpha = 1.0F - std::exp(-two_pi * options.rate_filter_hz * elapsed_seconds);
            filtered_rate_ += alpha * (pitch_rate_dps - filtered_rate_);
        }
        rate_initialized_ = true;
        previous_filter_hz_ = options.rate_filter_hz;
        const float error = target - measured;
        const float effective_kp = kp * proportionalScale(options, error, pitch_rate_dps);
        pid_.setTunings(effective_kp, ki, kd);
        const float limit = options.convergence ? std::clamp(available_pwm, 0.0F, 1000.0F) : 1000.0F;
        pid_.setAntiWindupOutputLimits(-limit, limit);
        const bool integrate = allowIntegration(options, error, pitch_rate_dps);
        const float pwm = options.gyro_damping
            ? pid_.updateWithMeasurementDelta(target, measured,
                filtered_rate_ * gain_reference_period_seconds, options.convergence, integrate)
            : pid_.update(target, measured, options.convergence, integrate);
        return {pwm, effective_kp, filtered_rate_, pid_.terms()};
    }

    void reset()
    {
        pid_.reset();
        filtered_rate_ = 0.0F;
        rate_initialized_ = false;
    }

private:
    PID pid_{0.0F, 0.0F, 0.0F, -1000.0F, 1000.0F, -100.0F, 100.0F};
    float filtered_rate_{};
    float previous_filter_hz_{};
    bool rate_initialized_{};
};

} // namespace balance_control
