#pragma once

#include <cmath>
#include <cstdint>

// Angles retain the MPU6050/body convention used by the balance controller.
// Pitch is already corrected by the current, height-dependent balance bias.
class BalanceStartupGate {
public:
    static constexpr std::uint16_t stable_samples_required = 50; // 500 ms at 100 Hz

    bool update(bool imu_valid, float pitch_degrees, float roll_degrees,
                float gyro_rate_degrees, float speed_target, float turn_target) noexcept
    {
        const bool valid = imu_valid && std::isfinite(pitch_degrees) &&
            std::isfinite(roll_degrees) && std::isfinite(gyro_rate_degrees) &&
            std::isfinite(speed_target) && std::isfinite(turn_target);
        if (!valid || std::fabs(pitch_degrees) > 30.0F || std::fabs(roll_degrees) > 30.0F) {
            reset();
            return false;
        }
        if (armed_) {
            return true;
        }

        const bool ready = std::fabs(pitch_degrees) <= 8.0F && std::fabs(roll_degrees) <= 5.0F &&
            std::fabs(gyro_rate_degrees) <= 20.0F &&
            std::fabs(speed_target) < 1.0F && std::fabs(turn_target) < 1.0F;
        if (!ready) {
            stable_samples_ = 0;
            return false;
        }
        if (++stable_samples_ >= stable_samples_required) {
            armed_ = true;
        }
        return armed_;
    }

    void reset() noexcept
    {
        armed_ = false;
        stable_samples_ = 0;
    }

private:
    bool armed_ = false;
    std::uint16_t stable_samples_ = 0;
};
