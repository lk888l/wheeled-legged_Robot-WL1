#pragma once

#include <cmath>

namespace BalanceSignal {

inline constexpr float nominal_angle_period_s = 0.01F;
inline constexpr float nominal_velocity_period_ms = 50.0F;
inline constexpr float wheel_filter_time_ms = 25.0F;

// ZYX Euler convention, as in MPU6050::QuatToEuler. gyro is bias-corrected
// body angular velocity (rad/s); a tilted chassis needs both Y and Z axes.
inline float pitchRate(float roll_degrees, const double gyro[3]) noexcept
{
    constexpr float radians = 0.017453292519943295F;
    const float roll = roll_degrees * radians;
    return (static_cast<float>(gyro[1]) * std::cos(roll) -
            static_cast<float>(gyro[2]) * std::sin(roll)) / radians;
}

class WheelFilter {
public:
    float update(float rpm, float elapsed_ms) noexcept
    {
        if (elapsed_ms > 0.0F) {
            value_ += elapsed_ms / (wheel_filter_time_ms + elapsed_ms) * (rpm - value_);
        }
        return value_;
    }
    void reset() noexcept { value_ = 0.0F; }
private:
    float value_{};
};

} // namespace BalanceSignal
