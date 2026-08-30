#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace wl1::control {

enum class ControlStrength : std::uint8_t
{
    gentle = 0,
    normal = 1,
    sport = 2,
};

struct PidGains
{
    float kp = 0.0F;
    float ki = 0.0F;
    float kd = 0.0F;
};

struct PidLimits
{
    float minimum_output = 0.0F;
    float maximum_output = 0.0F;
    float minimum_integral = 0.0F;
    float maximum_integral = 0.0F;
};

struct ControlLoopProfile
{
    PidGains gains{};
    PidLimits limits{};
};

struct ControlCommandLimits
{
    float maximum_angle_target_degrees = 0.0F;
    float maximum_velocity_target_rpm = 0.0F;
    float maximum_differential_target_rpm = 0.0F;
    float maximum_roll_target_degrees = 0.0F;
    float minimum_leg_height_mm = 44.5F;
    float maximum_leg_height_mm = 78.5F;
    float maximum_leg_rate_mm_per_second = 0.0F;
    float maximum_motor_pwm = 0.0F;
    float maximum_motor_pwm_slew_per_second = 0.0F;
};

/**
 * Complete tuning and authority limits for one operator-selectable control level.
 *
 * The pitch Kp used by the firmware changes with leg height. `angle.gains.kp`
 * is therefore the zero-height intercept and `angle_kp_per_mm` is the slope:
 *
 *     effective Kp = angle.gains.kp + angle_kp_per_mm * leg_height_mm
 */
struct ControlProfile
{
    ControlLoopProfile angle{};
    ControlLoopProfile velocity{};
    ControlLoopProfile differential{};
    ControlLoopProfile roll{};
    float angle_kp_per_mm = 0.0F;
    ControlCommandLimits commands{};
};

inline constexpr ControlProfile kGentleControlProfile{
    .angle = {{56.9F, 0.0F, 60.0F}, {-1000.0F, 1000.0F, -100.0F, 100.0F}},
    .velocity = {{0.0375F, 0.0060F, 0.0F}, {-7.0F, 7.0F, -75.0F, 75.0F}},
    .differential = {{1.5F, 0.00075F, 0.0F}, {-350.0F, 350.0F, -75.0F, 75.0F}},
    .roll = {{0.0F, -0.30F, 0.0F}, {-60.0F, 60.0F, -75.0F, 75.0F}},
    .angle_kp_per_mm = 0.30F,
    .commands = {
        .maximum_angle_target_degrees = 7.0F,
        .maximum_velocity_target_rpm = 65.0F,
        .maximum_differential_target_rpm = 65.0F,
        .maximum_roll_target_degrees = 12.0F,
        .minimum_leg_height_mm = 44.5F,
        .maximum_leg_height_mm = 78.5F,
        .maximum_leg_rate_mm_per_second = 80.0F,
        // Keep full inner-loop recovery authority; gentle mode is achieved by
        // reducing outer-loop gains and command limits.
        .maximum_motor_pwm = 1000.0F,
        // One 10 ms update may still traverse the complete +/-1000 range;
        // inner-loop recovery authority is intentionally not softened.
        .maximum_motor_pwm_slew_per_second = 200000.0F,
    },
};

inline constexpr ControlProfile kNormalControlProfile{
    .angle = {{56.9F, 0.0F, 60.0F}, {-1000.0F, 1000.0F, -100.0F, 100.0F}},
    .velocity = {{0.05F, 0.008F, 0.0F}, {-10.0F, 10.0F, -100.0F, 100.0F}},
    .differential = {{2.0F, 0.001F, 0.0F}, {-500.0F, 500.0F, -100.0F, 100.0F}},
    .roll = {{0.0F, -0.40F, 0.0F}, {-78.0F, 78.0F, -100.0F, 100.0F}},
    .angle_kp_per_mm = 0.30F,
    .commands = {
        .maximum_angle_target_degrees = 10.0F,
        .maximum_velocity_target_rpm = 100.0F,
        .maximum_differential_target_rpm = 100.0F,
        .maximum_roll_target_degrees = 18.0F,
        .minimum_leg_height_mm = 44.5F,
        .maximum_leg_height_mm = 78.5F,
        .maximum_leg_rate_mm_per_second = 140.0F,
        .maximum_motor_pwm = 1000.0F,
        .maximum_motor_pwm_slew_per_second = 200000.0F,
    },
};

inline constexpr ControlProfile kSportControlProfile{
    // Keep the already validated inner balance loop unchanged. Sport mode raises
    // outer-loop response and command authority only; inner-loop changes require
    // hardware calibration rather than an arbitrary global multiplier.
    .angle = {{56.9F, 0.0F, 60.0F}, {-1000.0F, 1000.0F, -100.0F, 100.0F}},
    .velocity = {{0.055F, 0.0085F, 0.0F}, {-10.0F, 10.0F, -100.0F, 100.0F}},
    .differential = {{2.2F, 0.0011F, 0.0F}, {-500.0F, 500.0F, -100.0F, 100.0F}},
    .roll = {{0.0F, -0.45F, 0.0F}, {-78.0F, 78.0F, -100.0F, 100.0F}},
    .angle_kp_per_mm = 0.30F,
    .commands = {
        .maximum_angle_target_degrees = 10.0F,
        .maximum_velocity_target_rpm = 100.0F,
        .maximum_differential_target_rpm = 100.0F,
        .maximum_roll_target_degrees = 18.0F,
        .minimum_leg_height_mm = 44.5F,
        .maximum_leg_height_mm = 78.5F,
        .maximum_leg_rate_mm_per_second = 180.0F,
        .maximum_motor_pwm = 1000.0F,
        .maximum_motor_pwm_slew_per_second = 200000.0F,
    },
};

[[nodiscard]] constexpr ControlStrength sanitizeControlStrength(ControlStrength strength) noexcept
{
    switch (strength)
    {
        case ControlStrength::gentle:
        case ControlStrength::normal:
        case ControlStrength::sport:
            return strength;
    }
    return ControlStrength::normal;
}

[[nodiscard]] constexpr const ControlProfile& controlProfile(ControlStrength strength) noexcept
{
    switch (sanitizeControlStrength(strength))
    {
        case ControlStrength::gentle:
            return kGentleControlProfile;
        case ControlStrength::sport:
            return kSportControlProfile;
        case ControlStrength::normal:
            return kNormalControlProfile;
    }
    return kNormalControlProfile;
}

[[nodiscard]] constexpr PidGains angleGainsForHeight(const ControlProfile& profile,
                                                      float leg_height_mm) noexcept
{
    PidGains result = profile.angle.gains;
    result.kp += profile.angle_kp_per_mm
        * std::clamp(leg_height_mm,
                     profile.commands.minimum_leg_height_mm,
                     profile.commands.maximum_leg_height_mm);
    return result;
}

namespace detail {

[[nodiscard]] constexpr float interpolate(float from, float to, float fraction) noexcept
{
    return from + ((to - from) * fraction);
}

[[nodiscard]] constexpr PidGains interpolate(const PidGains& from,
                                             const PidGains& to,
                                             float fraction) noexcept
{
    return {
        interpolate(from.kp, to.kp, fraction),
        interpolate(from.ki, to.ki, fraction),
        interpolate(from.kd, to.kd, fraction),
    };
}

[[nodiscard]] constexpr PidLimits interpolate(const PidLimits& from,
                                              const PidLimits& to,
                                              float fraction) noexcept
{
    return {
        interpolate(from.minimum_output, to.minimum_output, fraction),
        interpolate(from.maximum_output, to.maximum_output, fraction),
        interpolate(from.minimum_integral, to.minimum_integral, fraction),
        interpolate(from.maximum_integral, to.maximum_integral, fraction),
    };
}

[[nodiscard]] constexpr ControlLoopProfile interpolate(const ControlLoopProfile& from,
                                                       const ControlLoopProfile& to,
                                                       float fraction) noexcept
{
    return {
        interpolate(from.gains, to.gains, fraction),
        interpolate(from.limits, to.limits, fraction),
    };
}

[[nodiscard]] constexpr ControlCommandLimits interpolate(const ControlCommandLimits& from,
                                                         const ControlCommandLimits& to,
                                                         float fraction) noexcept
{
    return {
        interpolate(from.maximum_angle_target_degrees,
                    to.maximum_angle_target_degrees,
                    fraction),
        interpolate(from.maximum_velocity_target_rpm,
                    to.maximum_velocity_target_rpm,
                    fraction),
        interpolate(from.maximum_differential_target_rpm,
                    to.maximum_differential_target_rpm,
                    fraction),
        interpolate(from.maximum_roll_target_degrees,
                    to.maximum_roll_target_degrees,
                    fraction),
        interpolate(from.minimum_leg_height_mm, to.minimum_leg_height_mm, fraction),
        interpolate(from.maximum_leg_height_mm, to.maximum_leg_height_mm, fraction),
        interpolate(from.maximum_leg_rate_mm_per_second,
                    to.maximum_leg_rate_mm_per_second,
                    fraction),
        interpolate(from.maximum_motor_pwm, to.maximum_motor_pwm, fraction),
        interpolate(from.maximum_motor_pwm_slew_per_second,
                    to.maximum_motor_pwm_slew_per_second,
                    fraction),
    };
}

[[nodiscard]] constexpr ControlProfile interpolate(const ControlProfile& from,
                                                   const ControlProfile& to,
                                                   float fraction) noexcept
{
    return {
        interpolate(from.angle, to.angle, fraction),
        interpolate(from.velocity, to.velocity, fraction),
        interpolate(from.differential, to.differential, fraction),
        interpolate(from.roll, to.roll, fraction),
        interpolate(from.angle_kp_per_mm, to.angle_kp_per_mm, fraction),
        interpolate(from.commands, to.commands, fraction),
    };
}

} // namespace detail

/**
 * Produces a bounded, bumpless parameter trajectory between control levels.
 *
 * The class only interpolates parameters. The caller still owns PID state and
 * should prime derivative history when integrating the final profile switch.
 */
class ControlProfileTransition
{
public:
    static constexpr float default_duration_seconds = 0.40F;

    explicit ControlProfileTransition(
        ControlStrength initial = ControlStrength::normal,
        float duration_seconds = default_duration_seconds) noexcept
        : selected_(sanitizeControlStrength(initial)),
          target_(selected_),
          current_(controlProfile(selected_)),
          start_(current_),
          target_profile_(current_),
          duration_seconds_(validDuration(duration_seconds)
                                ? duration_seconds
                                : default_duration_seconds)
    {
    }

    void request(ControlStrength target) noexcept
    {
        target = sanitizeControlStrength(target);
        if (target == target_)
        {
            return;
        }

        start_ = current_;
        target_profile_ = controlProfile(target);
        target_ = target;
        elapsed_seconds_ = 0.0F;
        transitioning_ = true;
    }

    [[nodiscard]] const ControlProfile& advance(float elapsed_seconds) noexcept
    {
        if (!transitioning_ || !std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0F)
        {
            return current_;
        }

        elapsed_seconds_ = std::min(elapsed_seconds_ + elapsed_seconds, duration_seconds_);
        const float fraction = elapsed_seconds_ / duration_seconds_;
        current_ = detail::interpolate(start_, target_profile_, fraction);

        if (elapsed_seconds_ >= duration_seconds_)
        {
            current_ = target_profile_;
            selected_ = target_;
            transitioning_ = false;
        }
        return current_;
    }

    [[nodiscard]] const ControlProfile& current() const noexcept
    {
        return current_;
    }

    [[nodiscard]] ControlStrength selected() const noexcept
    {
        return selected_;
    }

    [[nodiscard]] ControlStrength target() const noexcept
    {
        return target_;
    }

    [[nodiscard]] bool transitioning() const noexcept
    {
        return transitioning_;
    }

    [[nodiscard]] float progress() const noexcept
    {
        return transitioning_ ? elapsed_seconds_ / duration_seconds_ : 1.0F;
    }

    [[nodiscard]] float durationSeconds() const noexcept
    {
        return duration_seconds_;
    }

private:
    [[nodiscard]] static bool validDuration(float duration_seconds) noexcept
    {
        return std::isfinite(duration_seconds) && duration_seconds > 0.0F;
    }

    ControlStrength selected_;
    ControlStrength target_;
    ControlProfile current_;
    ControlProfile start_;
    ControlProfile target_profile_;
    float duration_seconds_;
    float elapsed_seconds_ = 0.0F;
    bool transitioning_ = false;
};

} // namespace wl1::control
