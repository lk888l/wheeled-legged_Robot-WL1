#include "JumpController.hpp"

#include <algorithm>
#include <cmath>

namespace wl1::control {
namespace {

constexpr float kMaximumUpdateIntervalSeconds = 0.10F;

[[nodiscard]] bool finite(float value) noexcept
{
    return std::isfinite(value);
}

[[nodiscard]] bool finitePositive(float value) noexcept
{
    return finite(value) && value > 0.0F;
}

[[nodiscard]] float clamp(float value, float minimum, float maximum) noexcept
{
    return std::clamp(value, minimum, maximum);
}

} // namespace

JumpController::JumpController(JumpConfig config) noexcept
    : config_(config),
      saved_leg_height_mm_(config.minimum_leg_height_mm)
{
}

JumpOutput JumpController::update(const JumpInput& input) noexcept
{
    const bool request_edge = input.jump_request && !last_request_level_;
    last_request_level_ = input.jump_request;

    if (!input.controller_enabled)
    {
        const bool should_disarm = input.armed;
        reset(input.jump_request);
        JumpOutput output = takeOutput(input);
        output.disarm_requested = should_disarm;
        return output;
    }

    if (!configValid())
    {
        enterFault(JumpFault::invalid_config);
        return takeOutput(input);
    }
    if (!inputFinite(input))
    {
        enterFault(JumpFault::invalid_input);
        return takeOutput(input);
    }

    if (state_ == JumpState::fault)
    {
        return takeOutput(input);
    }

    if (state_ == JumpState::disabled)
    {
        // Do not latch a boot-time fault while waiting for the first valid radio
        // and IMU sample. Entering with an already asserted arm signal requires
        // an explicit disarm/re-arm cycle before a request can be accepted.
        if (!input.link_fresh || !input.imu_valid || !input.actuation_available
            || !input.timing_valid)
        {
            return takeOutput(input);
        }
        disarm_required_ = input.armed;
        transitionTo(JumpState::ready);
        return takeOutput(input);
    }

    if (!input.timing_valid)
    {
        enterFault(JumpFault::control_timing_violation);
        return takeOutput(input);
    }
    if (!input.actuation_available)
    {
        enterFault(JumpFault::servo_unavailable);
        return takeOutput(input);
    }

    if (!input.link_fresh)
    {
        enterFault(JumpFault::link_lost);
        return takeOutput(input);
    }
    if (!input.imu_valid)
    {
        enterFault(JumpFault::imu_invalid);
        return takeOutput(input);
    }

    if (actionState())
    {
        if (!input.armed)
        {
            enterFault(JumpFault::disarmed_during_action);
            return takeOutput(input);
        }
        if (std::fabs(input.roll_degrees) > config_.active_roll_abort_degrees)
        {
            enterFault(JumpFault::excessive_roll);
            return takeOutput(input);
        }
        if (std::fabs(input.pitch_error_degrees)
            > config_.active_pitch_error_abort_degrees)
        {
            enterFault(JumpFault::excessive_pitch);
            return takeOutput(input);
        }
        if (std::max(std::fabs(input.roll_rate_degrees_per_second),
                     input.gyro_norm_degrees_per_second)
            > config_.active_gyro_abort_degrees_per_second)
        {
            enterFault(JumpFault::excessive_angular_rate);
            return takeOutput(input);
        }
        if (input.wheel_speed_norm_rpm > config_.active_wheel_speed_abort_rpm)
        {
            enterFault(JumpFault::excessive_wheel_speed);
            return takeOutput(input);
        }
        if (input.acceleration_norm_g > config_.maximum_measurable_acceleration_g)
        {
            enterFault(JumpFault::acceleration_out_of_range);
            return takeOutput(input);
        }
        if (!input.controls_neutral)
        {
            enterFault(JumpFault::controls_not_neutral);
            return takeOutput(input);
        }
    }

    state_elapsed_seconds_ += input.elapsed_seconds;

    switch (state_)
    {
        case JumpState::ready:
            if (!input.armed)
            {
                disarm_required_ = false;
            }

            if (readyObservation(input))
            {
                stable_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                stable_elapsed_seconds_ = 0.0F;
            }

            if (request_edge)
            {
                if (!input.armed || disarm_required_)
                {
                    enterFault(JumpFault::request_not_armed);
                }
                else if (stable_elapsed_seconds_ < config_.ready_stable_time_seconds)
                {
                    enterFault(JumpFault::unstable_request);
                }
                else
                {
                    saved_leg_height_mm_ = clamp(input.requested_leg_height_mm,
                                                 config_.minimum_leg_height_mm,
                                                 config_.maximum_leg_height_mm);
                    transitionTo(JumpState::preload, true);
                }
            }
            break;

        case JumpState::preload:
            if (readyObservation(input))
            {
                stable_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                stable_elapsed_seconds_ = 0.0F;
            }

            if (state_elapsed_seconds_ >= config_.preload_hold_seconds
                && stable_elapsed_seconds_ >= config_.preload_settle_seconds)
            {
                transitionTo(JumpState::thrust);
            }
            else if (state_elapsed_seconds_ >= config_.preload_timeout_seconds)
            {
                enterFault(JumpFault::phase_timeout);
            }
            break;

        case JumpState::thrust:
            if (input.acceleration_norm_g >= config_.launch_acceleration_g)
            {
                launch_elapsed_seconds_ += input.elapsed_seconds;
                if (launch_elapsed_seconds_ >= config_.event_debounce_seconds)
                {
                    launch_seen_ = true;
                }
            }
            else
            {
                launch_elapsed_seconds_ = 0.0F;
            }

            if (launch_seen_
                && state_elapsed_seconds_ >= config_.thrust_minimum_seconds
                && input.acceleration_norm_g <= config_.freefall_acceleration_g)
            {
                freefall_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                freefall_elapsed_seconds_ = 0.0F;
            }

            if (freefall_elapsed_seconds_ >= config_.event_debounce_seconds)
            {
                transitionTo(JumpState::flight);
            }
            else if (state_elapsed_seconds_ >= config_.thrust_timeout_seconds)
            {
                enterFault(JumpFault::phase_timeout);
            }
            break;

        case JumpState::flight:
            if (state_elapsed_seconds_ >= config_.flight_minimum_seconds
                && input.acceleration_norm_g >= config_.landing_impact_acceleration_g)
            {
                impact_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                impact_elapsed_seconds_ = 0.0F;
            }

            if (impact_elapsed_seconds_ >= config_.event_debounce_seconds)
            {
                transitionTo(JumpState::landing, true);
            }
            else if (state_elapsed_seconds_ >= config_.flight_timeout_seconds)
            {
                enterFault(JumpFault::phase_timeout);
            }
            break;

        case JumpState::landing:
            if (readyObservation(input))
            {
                stable_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                stable_elapsed_seconds_ = 0.0F;
            }

            if (state_elapsed_seconds_ >= config_.landing_minimum_seconds
                && stable_elapsed_seconds_ >= config_.landing_settle_seconds)
            {
                transitionTo(JumpState::recover, true);
            }
            else if (state_elapsed_seconds_ >= config_.landing_timeout_seconds)
            {
                enterFault(JumpFault::phase_timeout);
            }
            break;

        case JumpState::recover:
            if (readyObservation(input))
            {
                stable_elapsed_seconds_ += input.elapsed_seconds;
            }
            else
            {
                stable_elapsed_seconds_ = 0.0F;
            }

            if (state_elapsed_seconds_ >= config_.recover_motion_seconds
                && stable_elapsed_seconds_ >= config_.recover_settle_seconds)
            {
                disarm_required_ = true;
                transitionTo(JumpState::ready, true);
            }
            else if (state_elapsed_seconds_ >= config_.recover_timeout_seconds)
            {
                enterFault(JumpFault::phase_timeout);
            }
            break;

        case JumpState::disabled:
        case JumpState::fault:
            break;
    }

    return takeOutput(input);
}

void JumpController::reset(bool current_request_level) noexcept
{
    state_ = JumpState::disabled;
    fault_ = JumpFault::none;
    state_elapsed_seconds_ = 0.0F;
    stable_elapsed_seconds_ = 0.0F;
    launch_elapsed_seconds_ = 0.0F;
    freefall_elapsed_seconds_ = 0.0F;
    impact_elapsed_seconds_ = 0.0F;
    saved_leg_height_mm_ = config_.minimum_leg_height_mm;
    launch_seen_ = false;
    last_request_level_ = current_request_level;
    disarm_required_ = false;
    reset_control_requested_ = false;
    abort_pose_required_ = false;
}

JumpState JumpController::state() const noexcept
{
    return state_;
}

JumpFault JumpController::fault() const noexcept
{
    return fault_;
}

const JumpConfig& JumpController::config() const noexcept
{
    return config_;
}

bool JumpController::configValid() const noexcept
{
    const bool heights_valid = finite(config_.minimum_leg_height_mm)
        && finite(config_.maximum_leg_height_mm)
        && config_.minimum_leg_height_mm < config_.maximum_leg_height_mm;
    if (!heights_valid)
    {
        return false;
    }

    const auto height_in_range = [this](float height) {
        return finite(height) && height >= config_.minimum_leg_height_mm
            && height <= config_.maximum_leg_height_mm;
    };
    if (!height_in_range(config_.preload_height_mm)
        || !height_in_range(config_.thrust_height_mm)
        || !height_in_range(config_.flight_height_mm)
        || !height_in_range(config_.landing_height_mm)
        || !height_in_range(config_.abort_height_mm))
    {
        return false;
    }

    if (!finitePositive(config_.ready_roll_limit_degrees)
        || !finitePositive(config_.ready_pitch_error_limit_degrees)
        || !finitePositive(config_.ready_gyro_limit_degrees_per_second)
        || !finitePositive(config_.ready_wheel_speed_limit_rpm)
        || !finitePositive(config_.ready_acceleration_tolerance_g)
        || config_.ready_acceleration_tolerance_g >= 1.0F
        || !finitePositive(config_.ready_stable_time_seconds)
        || !finitePositive(config_.active_roll_abort_degrees)
        || config_.active_roll_abort_degrees <= config_.ready_roll_limit_degrees
        || !finitePositive(config_.active_pitch_error_abort_degrees)
        || config_.active_pitch_error_abort_degrees
            <= config_.ready_pitch_error_limit_degrees
        || !finitePositive(config_.active_gyro_abort_degrees_per_second)
        || config_.active_gyro_abort_degrees_per_second
            <= config_.ready_gyro_limit_degrees_per_second
        || !finitePositive(config_.active_wheel_speed_abort_rpm)
        || config_.active_wheel_speed_abort_rpm <= config_.ready_wheel_speed_limit_rpm)
    {
        return false;
    }

    if (!finitePositive(config_.freefall_acceleration_g)
        || config_.freefall_acceleration_g >= 1.0F
        || !finitePositive(config_.launch_acceleration_g)
        || config_.launch_acceleration_g <= 1.0F
        || !finitePositive(config_.landing_impact_acceleration_g)
        || config_.landing_impact_acceleration_g <= config_.launch_acceleration_g
        || !finitePositive(config_.maximum_measurable_acceleration_g)
        || config_.maximum_measurable_acceleration_g
            <= config_.landing_impact_acceleration_g
        || !finitePositive(config_.event_debounce_seconds))
    {
        return false;
    }

    const bool stage_times_valid = finitePositive(config_.preload_hold_seconds)
        && finitePositive(config_.preload_settle_seconds)
        && finitePositive(config_.preload_timeout_seconds)
        && config_.preload_hold_seconds < config_.preload_timeout_seconds
        && config_.preload_settle_seconds < config_.preload_timeout_seconds
        && finitePositive(config_.thrust_minimum_seconds)
        && finitePositive(config_.thrust_timeout_seconds)
        && config_.thrust_minimum_seconds < config_.thrust_timeout_seconds
        && finitePositive(config_.flight_minimum_seconds)
        && finitePositive(config_.flight_timeout_seconds)
        && config_.flight_minimum_seconds < config_.flight_timeout_seconds
        && finitePositive(config_.landing_minimum_seconds)
        && finitePositive(config_.landing_settle_seconds)
        && finitePositive(config_.landing_timeout_seconds)
        && config_.landing_minimum_seconds < config_.landing_timeout_seconds
        && config_.landing_settle_seconds < config_.landing_timeout_seconds
        && finitePositive(config_.recover_motion_seconds)
        && finitePositive(config_.recover_settle_seconds)
        && finitePositive(config_.recover_timeout_seconds)
        && config_.recover_motion_seconds < config_.recover_timeout_seconds
        && config_.recover_settle_seconds < config_.recover_timeout_seconds;
    if (!stage_times_valid)
    {
        return false;
    }

    return finite(config_.roll_trim_kp_mm_per_degree)
        && config_.roll_trim_kp_mm_per_degree >= 0.0F
        && finite(config_.roll_trim_kd_mm_per_degree_per_second)
        && config_.roll_trim_kd_mm_per_degree_per_second >= 0.0F
        && finite(config_.maximum_roll_trim_mm)
        && config_.maximum_roll_trim_mm >= 0.0F;
}

bool JumpController::inputFinite(const JumpInput& input) const noexcept
{
    return finitePositive(input.elapsed_seconds)
        && input.elapsed_seconds <= kMaximumUpdateIntervalSeconds
        && finite(input.roll_degrees)
        && finite(input.pitch_error_degrees)
        && finite(input.roll_rate_degrees_per_second)
        && finite(input.gyro_norm_degrees_per_second)
        && input.gyro_norm_degrees_per_second >= 0.0F
        && finite(input.wheel_speed_norm_rpm)
        && input.wheel_speed_norm_rpm >= 0.0F
        && finite(input.acceleration_norm_g)
        && input.acceleration_norm_g >= 0.0F
        && finite(input.requested_leg_height_mm);
}

bool JumpController::readyObservation(const JumpInput& input) const noexcept
{
    return input.actuation_available && input.actuation_settled && input.timing_valid
        && std::fabs(input.roll_degrees) <= config_.ready_roll_limit_degrees
        && std::fabs(input.pitch_error_degrees)
            <= config_.ready_pitch_error_limit_degrees
        && std::max(std::fabs(input.roll_rate_degrees_per_second),
                    input.gyro_norm_degrees_per_second)
            <= config_.ready_gyro_limit_degrees_per_second
        && input.wheel_speed_norm_rpm <= config_.ready_wheel_speed_limit_rpm
        && std::fabs(input.acceleration_norm_g - 1.0F)
            <= config_.ready_acceleration_tolerance_g
        && input.controls_neutral;
}

bool JumpController::actionState() const noexcept
{
    return state_ == JumpState::preload || state_ == JumpState::thrust
        || state_ == JumpState::flight || state_ == JumpState::landing
        || state_ == JumpState::recover;
}

void JumpController::transitionTo(JumpState state, bool reset_control_state) noexcept
{
    state_ = state;
    state_elapsed_seconds_ = 0.0F;
    stable_elapsed_seconds_ = 0.0F;
    launch_elapsed_seconds_ = 0.0F;
    freefall_elapsed_seconds_ = 0.0F;
    impact_elapsed_seconds_ = 0.0F;
    launch_seen_ = false;
    reset_control_requested_ = reset_control_requested_ || reset_control_state;
}

void JumpController::enterFault(JumpFault fault) noexcept
{
    if (state_ == JumpState::fault)
    {
        return;
    }
    // Only an action that has actually begun may request the conservative abort
    // pose. Configuration/link faults while idle must leave the real legs alone.
    abort_pose_required_ = abort_pose_required_ || actionState();
    fault_ = fault;
    disarm_required_ = true;
    transitionTo(JumpState::fault, true);
}

JumpOutput JumpController::takeOutput(const JumpInput& input) noexcept
{
    JumpOutput output;
    output.state = state_;
    output.fault = fault_;
    output.dry_run = !config_.actuation_enabled;
    output.action_active = actionState();
    output.freeze_outer_loops = output.action_active || state_ == JumpState::fault;
    output.reset_control_state = reset_control_requested_;
    output.disarm_requested = disarm_required_ || state_ == JumpState::fault;
    output.state_elapsed_seconds = state_elapsed_seconds_;
    output.ready_to_jump = state_ == JumpState::ready && input.armed
        && !disarm_required_
        && stable_elapsed_seconds_ >= config_.ready_stable_time_seconds;

    const float requested_height = clamp(input.requested_leg_height_mm,
                                         config_.minimum_leg_height_mm,
                                         config_.maximum_leg_height_mm);
    output.common_leg_height_mm = requested_height;

    switch (state_)
    {
        case JumpState::preload:
            output.leg_command_valid = true;
            output.common_leg_height_mm = config_.preload_height_mm;
            break;
        case JumpState::thrust:
            output.leg_command_valid = true;
            output.common_leg_height_mm = config_.thrust_height_mm;
            break;
        case JumpState::flight:
            output.leg_command_valid = true;
            output.common_leg_height_mm = config_.flight_height_mm;
            break;
        case JumpState::landing:
            output.leg_command_valid = true;
            output.common_leg_height_mm = config_.landing_height_mm;
            break;
        case JumpState::recover:
        {
            output.leg_command_valid = true;
            const float fraction = clamp(
                state_elapsed_seconds_ / config_.recover_motion_seconds, 0.0F, 1.0F);
            output.common_leg_height_mm = config_.landing_height_mm
                + ((saved_leg_height_mm_ - config_.landing_height_mm) * fraction);
            break;
        }
        case JumpState::fault:
            if (abort_pose_required_)
            {
                output.leg_command_valid = true;
                output.common_leg_height_mm = config_.abort_height_mm;
            }
            break;
        case JumpState::disabled:
        case JumpState::ready:
            break;
    }

    float roll_trim_mm = 0.0F;
    if (output.leg_command_valid && state_ != JumpState::fault)
    {
        roll_trim_mm = -(config_.roll_trim_kp_mm_per_degree * input.roll_degrees)
            - (config_.roll_trim_kd_mm_per_degree_per_second
               * input.roll_rate_degrees_per_second);
        roll_trim_mm = clamp(roll_trim_mm,
                             -config_.maximum_roll_trim_mm,
                             config_.maximum_roll_trim_mm);
    }

    output.left_leg_height_mm = clamp(output.common_leg_height_mm - roll_trim_mm,
                                      config_.minimum_leg_height_mm,
                                      config_.maximum_leg_height_mm);
    output.right_leg_height_mm = clamp(output.common_leg_height_mm + roll_trim_mm,
                                       config_.minimum_leg_height_mm,
                                       config_.maximum_leg_height_mm);
    output.apply_leg_command = output.leg_command_valid && config_.actuation_enabled;

    reset_control_requested_ = false;
    return output;
}

} // namespace wl1::control
