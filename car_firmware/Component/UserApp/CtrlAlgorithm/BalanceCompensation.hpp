#pragma once

#include <bit>
#include <cstdint>

namespace BalanceCompensation {

inline constexpr float minimum_leg_height_mm = 44.5F;
inline constexpr float maximum_leg_height_mm = 78.5F;
inline constexpr float default_minimum_bias_degrees = 9.5F;

constexpr float clampLegHeight(float height_mm) noexcept
{
    return height_mm < minimum_leg_height_mm ? minimum_leg_height_mm
         : height_mm > maximum_leg_height_mm ? maximum_leg_height_mm
         : height_mm;
}

// Inputs are the clamped targets shared with the servo task, not measured heights.
constexpr float averageLegHeight(float left_mm, float right_mm) noexcept
{
    return (left_mm + right_mm) / 2.0F;
}

constexpr float pitchBias(float minimum_bias_degrees, float average_height_mm) noexcept
{
    // minimum_bias + f(h) - f(h_min), f(h) = 0.01026*h*h - 1.258*h + 48.24.
    // Factoring the difference makes the height correction exactly zero at h_min.
    const float height_delta = average_height_mm - minimum_leg_height_mm;
    const float correction = height_delta *
        (0.01026F * (average_height_mm + minimum_leg_height_mm) - 1.258F);
    return minimum_bias_degrees + correction;
}

constexpr bool isFiniteBias(float bias_degrees) noexcept
{
    // Inspect IEEE bits so command validation does not depend on math optimizations.
    return (std::bit_cast<std::uint32_t>(bias_degrees) & 0x7F800000U) != 0x7F800000U;
}

} // namespace BalanceCompensation
