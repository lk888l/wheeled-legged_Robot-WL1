#pragma once

#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include <array>
#include <bit>
#include <cstdint>

namespace MotionSettings {

struct PidGains {
    float kp, ki, kd;
};

inline constexpr float reference_height_mm = 61.5F;
inline constexpr float angle_kp_height_slope = 0.3F;

// Only tunings and posture targets belong here. Speed, turn and motor commands
// deliberately start at zero after every boot.
struct Parameters {
    float minimum_pitch_bias = BalanceCompensation::default_minimum_bias_degrees;
    PidGains angle{75.35F, 0.0F, 60.0F}; // Original 0.3*h + 56.9 at h = 61.5 mm.
    PidGains velocity{0.05F, 0.008F, 0.0F};
    PidGains difference{2.0F, 0.001F, 0.0F};
    PidGains roll{0.0F, -0.4F, 0.0F};
    float leg_height = BalanceCompensation::minimum_leg_height_mm;
    float roll_target = 0.0F;
};

constexpr float effectiveAngleKp(float reference_kp, float average_height) noexcept
{
    return reference_kp + angle_kp_height_slope * (average_height - reference_height_mm);
}

inline constexpr std::size_t parameter_count = 15;
using ParameterWords = std::array<std::uint32_t, parameter_count>;

// Explicit field order is the version-1 storage format, independent of padding.
constexpr ParameterWords encode(const Parameters& p) noexcept
{
    const std::array values{p.minimum_pitch_bias, p.angle.kp, p.angle.ki, p.angle.kd,
        p.velocity.kp, p.velocity.ki, p.velocity.kd,
        p.difference.kp, p.difference.ki, p.difference.kd,
        p.roll.kp, p.roll.ki, p.roll.kd, p.leg_height, p.roll_target};
    return std::bit_cast<ParameterWords>(values);
}

constexpr Parameters decode(const ParameterWords& words) noexcept
{
    const auto v = std::bit_cast<std::array<float, parameter_count>>(words);
    return {v[0], {v[1], v[2], v[3]}, {v[4], v[5], v[6]},
        {v[7], v[8], v[9]}, {v[10], v[11], v[12]}, v[13], v[14]};
}

constexpr bool valid(const Parameters& p) noexcept
{
    for (const auto word : encode(p)) {
        if ((word & 0x7F800000U) == 0x7F800000U) return false;
    }
    return p.leg_height >= BalanceCompensation::minimum_leg_height_mm &&
        p.leg_height <= BalanceCompensation::maximum_leg_height_mm;
}

} // namespace MotionSettings
