#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "ControlProfile.hpp"
#include "JumpController.hpp"

namespace {

using wl1::control::ControlProfileTransition;
using wl1::control::ControlStrength;
using wl1::control::JumpConfig;
using wl1::control::JumpController;
using wl1::control::JumpFault;
using wl1::control::JumpInput;
using wl1::control::JumpOutput;
using wl1::control::JumpState;

static_assert(static_cast<std::uint8_t>(JumpFault::servo_unavailable) == 5U);
static_assert(static_cast<std::uint8_t>(JumpFault::control_timing_violation) == 6U);

[[nodiscard]] bool nearlyEqual(float left, float right, float tolerance = 1.0e-5F)
{
    return std::fabs(left - right) <= tolerance;
}

[[nodiscard]] JumpInput stableInput()
{
    return JumpInput{
        .elapsed_seconds = 0.01F,
        .controller_enabled = true,
        .armed = false,
        .jump_request = false,
        .imu_valid = true,
        .link_fresh = true,
        .actuation_available = true,
        .actuation_settled = true,
        .timing_valid = true,
        .roll_degrees = 0.0F,
        .pitch_error_degrees = 0.0F,
        .roll_rate_degrees_per_second = 0.0F,
        .gyro_norm_degrees_per_second = 0.0F,
        .wheel_speed_norm_rpm = 0.0F,
        .acceleration_norm_g = 1.0F,
        .controls_neutral = true,
        .requested_leg_height_mm = 61.5F,
    };
}

[[nodiscard]] JumpOutput repeat(JumpController& controller,
                                JumpInput& input,
                                std::uint32_t count)
{
    JumpOutput output;
    for (std::uint32_t index = 0; index < count; ++index)
    {
        output = controller.update(input);
    }
    return output;
}

[[nodiscard]] JumpOutput repeatUntilState(JumpController& controller,
                                         JumpInput& input,
                                         JumpState expected,
                                         std::uint32_t maximum_count)
{
    JumpOutput output;
    for (std::uint32_t index = 0; index < maximum_count; ++index)
    {
        output = controller.update(input);
        if (output.state == expected)
        {
            return output;
        }
    }
    assert(false && "jump state transition timed out in host test");
    return output;
}

[[nodiscard]] JumpOutput armAndRequest(JumpController& controller, JumpInput& input)
{
    JumpOutput output = controller.update(input);
    assert(output.state == JumpState::ready);

    output = repeat(controller, input, 25U);
    input.armed = true;
    output = controller.update(input);
    assert(output.ready_to_jump);

    input.jump_request = true;
    output = controller.update(input);
    input.jump_request = false;
    assert(output.state == JumpState::preload);
    return output;
}

void testControlProfilesContainCompleteBoundedTunings()
{
    const auto& gentle = wl1::control::controlProfile(ControlStrength::gentle);
    const auto& normal = wl1::control::controlProfile(ControlStrength::normal);
    const auto& sport = wl1::control::controlProfile(ControlStrength::sport);

    assert(nearlyEqual(normal.velocity.gains.kp, 0.05F));
    assert(nearlyEqual(normal.velocity.gains.ki, 0.008F));
    assert(nearlyEqual(normal.differential.gains.kp, 2.0F));
    assert(nearlyEqual(normal.roll.gains.ki, -0.4F));
    assert(nearlyEqual(normal.angle.limits.maximum_output, 1000.0F));
    assert(nearlyEqual(normal.commands.minimum_leg_height_mm, 44.5F));
    assert(nearlyEqual(normal.commands.maximum_leg_height_mm, 78.5F));

    const auto low_height_gains = wl1::control::angleGainsForHeight(normal, 44.5F);
    const auto high_height_gains = wl1::control::angleGainsForHeight(normal, 78.5F);
    assert(nearlyEqual(low_height_gains.kp, (0.3F * 44.5F) + 56.9F));
    assert(nearlyEqual(high_height_gains.kp, (0.3F * 78.5F) + 56.9F));
    assert(nearlyEqual(low_height_gains.kd, 60.0F));

    assert(gentle.velocity.gains.kp < normal.velocity.gains.kp);
    assert(sport.velocity.gains.kp > normal.velocity.gains.kp);
    assert(gentle.commands.maximum_velocity_target_rpm
           < normal.commands.maximum_velocity_target_rpm);
    assert(gentle.commands.maximum_motor_pwm_slew_per_second >= 200000.0F);
    assert(normal.commands.maximum_motor_pwm_slew_per_second >= 200000.0F);
    assert(sport.commands.maximum_motor_pwm_slew_per_second >= 200000.0F);

    const auto invalid = static_cast<ControlStrength>(255U);
    assert(&wl1::control::controlProfile(invalid) == &normal);
}

void testControlProfileTransitionIsLinearAndRetargetable()
{
    ControlProfileTransition transition(ControlStrength::normal);
    transition.request(ControlStrength::gentle);
    assert(transition.transitioning());
    assert(transition.target() == ControlStrength::gentle);

    const auto& halfway = transition.advance(0.20F);
    const float expected_half_velocity_kp = (0.05F + 0.0375F) * 0.5F;
    assert(nearlyEqual(halfway.velocity.gains.kp, expected_half_velocity_kp));
    assert(nearlyEqual(transition.progress(), 0.5F));
    assert(transition.selected() == ControlStrength::normal);

    const float value_before_retarget = transition.current().velocity.gains.kp;
    transition.request(ControlStrength::sport);
    assert(nearlyEqual(transition.current().velocity.gains.kp, value_before_retarget));
    const auto& retargeted = transition.advance(0.20F);
    assert(nearlyEqual(retargeted.velocity.gains.kp,
                       (value_before_retarget + 0.055F) * 0.5F));

    const auto& completed = transition.advance(0.20F);
    assert(!transition.transitioning());
    assert(transition.selected() == ControlStrength::sport);
    assert(nearlyEqual(completed.velocity.gains.kp, 0.055F));
    assert(nearlyEqual(transition.progress(), 1.0F));

    const float completed_value = completed.velocity.gains.kp;
    static_cast<void>(transition.advance(-1.0F));
    static_cast<void>(transition.advance(NAN));
    assert(nearlyEqual(transition.current().velocity.gains.kp, completed_value));
}

void testJumpDefaultsToDryRunAndRequiresStableArming()
{
    JumpController controller;
    JumpInput input = stableInput();

    input.link_fresh = false;
    JumpOutput output = controller.update(input);
    assert(output.state == JumpState::disabled);
    assert(output.fault == JumpFault::none);

    input.link_fresh = true;
    output = armAndRequest(controller, input);
    assert(output.dry_run);
    assert(output.action_active);
    assert(output.leg_command_valid);
    assert(!output.apply_leg_command);
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    assert(nearlyEqual(output.common_leg_height_mm,
                       controller.config().preload_height_mm));
}

void testJumpNominalPhaseSequenceAndOneShotDisarm()
{
    JumpController controller;
    JumpInput input = stableInput();
    JumpOutput output = armAndRequest(controller, input);

    output = repeatUntilState(controller, input, JumpState::thrust, 70U);
    assert(nearlyEqual(output.common_leg_height_mm,
                       controller.config().thrust_height_mm));

    input.acceleration_norm_g = 1.20F;
    output = repeat(controller, input, 3U);
    assert(output.state == JumpState::thrust);
    input.acceleration_norm_g = 1.0F;
    output = repeat(controller, input, 3U);
    input.acceleration_norm_g = 0.30F;
    output = repeatUntilState(controller, input, JumpState::flight, 5U);
    assert(nearlyEqual(output.common_leg_height_mm,
                       controller.config().flight_height_mm));

    input.acceleration_norm_g = 0.30F;
    output = repeat(controller, input, 4U);
    input.acceleration_norm_g = 1.50F;
    output = repeatUntilState(controller, input, JumpState::landing, 5U);
    assert(output.reset_control_state);
    assert(nearlyEqual(output.common_leg_height_mm,
                       controller.config().landing_height_mm));

    input.acceleration_norm_g = 1.0F;
    output = repeatUntilState(controller, input, JumpState::recover, 20U);
    assert(output.reset_control_state);
    assert(output.leg_command_valid);

    output = repeatUntilState(controller, input, JumpState::ready, 60U);
    assert(output.disarm_requested);
    assert(output.reset_control_state);
    assert(!output.action_active);
    assert(!output.ready_to_jump);

    // A consumed arm cycle cannot trigger another action. The request must drop,
    // the operator must disarm, and only then may a new arm cycle begin.
    input.jump_request = true;
    output = controller.update(input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::request_not_armed);
    assert(output.disarm_requested);

    input.controller_enabled = false;
    input.armed = false;
    input.jump_request = false;
    output = controller.update(input);
    assert(output.state == JumpState::disabled);
    assert(output.fault == JumpFault::none);
}

void testJumpRollTrimIsBoundedAndActuationMustBeExplicit()
{
    JumpConfig config;
    config.actuation_enabled = true;
    JumpController controller(config);
    JumpInput input = stableInput();
    JumpOutput output = armAndRequest(controller, input);

    input.roll_degrees = 4.0F;
    input.roll_rate_degrees_per_second = 10.0F;
    input.gyro_norm_degrees_per_second = 10.0F;
    output = controller.update(input);
    assert(!output.dry_run);
    assert(output.apply_leg_command);
    assert(output.left_leg_height_mm >= config.minimum_leg_height_mm);
    assert(output.right_leg_height_mm >= config.minimum_leg_height_mm);
    assert(output.left_leg_height_mm <= config.maximum_leg_height_mm);
    assert(output.right_leg_height_mm <= config.maximum_leg_height_mm);
    assert(std::fabs(output.left_leg_height_mm - output.right_leg_height_mm)
           <= (2.0F * config.maximum_roll_trim_mm) + 1.0e-5F);
}

void testJumpReadyRequiresPitchWheelAndNeutralControls()
{
    const auto expect_unstable_request = [](auto make_unstable) {
        JumpController controller;
        JumpInput input = stableInput();
        JumpOutput output = controller.update(input);
        assert(output.state == JumpState::ready);
        output = repeat(controller, input, 25U);

        input.armed = true;
        output = controller.update(input);
        assert(output.ready_to_jump);

        make_unstable(input, controller.config());
        input.jump_request = true;
        output = controller.update(input);
        assert(output.state == JumpState::fault);
        assert(output.fault == JumpFault::unstable_request);
        // A request rejected before an action starts must never move a real leg.
        assert(!output.leg_command_valid);
        assert(!output.apply_leg_command);
    };

    expect_unstable_request([](JumpInput& input, const JumpConfig& config) {
        input.pitch_error_degrees = config.ready_pitch_error_limit_degrees + 0.1F;
    });
    expect_unstable_request([](JumpInput& input, const JumpConfig& config) {
        input.wheel_speed_norm_rpm = config.ready_wheel_speed_limit_rpm + 0.1F;
    });
    expect_unstable_request([](JumpInput& input, const JumpConfig&) {
        input.controls_neutral = false;
    });
}

void testJumpActionFaultCommandsBoundedAbortPoseAndStaysLatched()
{
    JumpConfig active_config;
    active_config.actuation_enabled = true;
    JumpController pitch_controller(active_config);
    JumpInput pitch_input = stableInput();
    JumpOutput output = armAndRequest(pitch_controller, pitch_input);
    assert(output.state == JumpState::preload);

    pitch_input.roll_degrees = 4.0F;
    pitch_input.roll_rate_degrees_per_second = 10.0F;
    pitch_input.gyro_norm_degrees_per_second = 10.0F;
    pitch_input.pitch_error_degrees = active_config.active_pitch_error_abort_degrees + 0.1F;
    output = pitch_controller.update(pitch_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::excessive_pitch);
    assert(output.leg_command_valid);
    assert(output.apply_leg_command);
    assert(nearlyEqual(output.common_leg_height_mm, active_config.abort_height_mm));
    // Abort is deliberately symmetric; potentially invalid attitude feedback
    // must not trim the conservative fallback target.
    assert(nearlyEqual(output.left_leg_height_mm, active_config.abort_height_mm));
    assert(nearlyEqual(output.right_leg_height_mm, active_config.abort_height_mm));
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    assert(output.disarm_requested);

    pitch_input.pitch_error_degrees = 0.0F;
    pitch_input.armed = false;
    output = pitch_controller.update(pitch_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::excessive_pitch);
    assert(output.apply_leg_command);
    assert(nearlyEqual(output.common_leg_height_mm, active_config.abort_height_mm));

    pitch_controller.reset(false);
    output = pitch_controller.update(pitch_input);
    assert(output.state == JumpState::ready);
    assert(output.fault == JumpFault::none);
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);

    JumpController wheel_controller(active_config);
    JumpInput wheel_input = stableInput();
    output = armAndRequest(wheel_controller, wheel_input);
    wheel_input.wheel_speed_norm_rpm = active_config.active_wheel_speed_abort_rpm + 0.1F;
    output = wheel_controller.update(wheel_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::excessive_wheel_speed);
    assert(output.apply_leg_command);

    JumpController dry_run_controller;
    JumpInput controls_input = stableInput();
    output = armAndRequest(dry_run_controller, controls_input);
    controls_input.controls_neutral = false;
    output = dry_run_controller.update(controls_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::controls_not_neutral);
    assert(output.leg_command_valid);
    assert(output.dry_run);
    assert(!output.apply_leg_command);
    assert(nearlyEqual(output.common_leg_height_mm,
                       dry_run_controller.config().abort_height_mm));

    JumpController idle_fault_controller(active_config);
    JumpInput idle_input = stableInput();
    output = idle_fault_controller.update(idle_input);
    assert(output.state == JumpState::ready);
    idle_input.link_fresh = false;
    output = idle_fault_controller.update(idle_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::link_lost);
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);
}

void testJumpFaultsAreLatchedAndPhasesHaveTimeouts()
{
    JumpController timeout_controller;
    JumpInput timeout_input = stableInput();
    JumpOutput output = armAndRequest(timeout_controller, timeout_input);
    output = repeatUntilState(timeout_controller, timeout_input, JumpState::thrust, 70U);
    timeout_input.acceleration_norm_g = 1.0F;
    output = repeatUntilState(timeout_controller, timeout_input, JumpState::fault, 40U);
    assert(output.fault == JumpFault::phase_timeout);
    assert(output.disarm_requested);
    assert(output.freeze_outer_loops);

    timeout_input.acceleration_norm_g = 0.2F;
    output = timeout_controller.update(timeout_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::phase_timeout);

    JumpController link_controller;
    JumpInput link_input = stableInput();
    output = link_controller.update(link_input);
    assert(output.state == JumpState::ready);
    link_input.link_fresh = false;
    output = link_controller.update(link_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::link_lost);

    JumpController imu_controller;
    JumpInput imu_input = stableInput();
    output = imu_controller.update(imu_input);
    assert(output.state == JumpState::ready);
    imu_input.imu_valid = false;
    output = imu_controller.update(imu_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::imu_invalid);

    JumpController disarm_controller;
    JumpInput disarm_input = stableInput();
    output = armAndRequest(disarm_controller, disarm_input);
    disarm_input.armed = false;
    output = disarm_controller.update(disarm_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::disarmed_during_action);
}

void testJumpActuationAvailabilityInterlocksDisabledReadyAndAction()
{
    JumpController disabled_controller;
    JumpInput disabled_input = stableInput();
    disabled_input.actuation_available = false;

    JumpOutput output = repeat(disabled_controller, disabled_input, 25U);
    assert(output.state == JumpState::disabled);
    assert(output.fault == JumpFault::none);
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);

    disabled_input.actuation_available = true;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::ready);
    assert(output.fault == JumpFault::none);

    disabled_input.actuation_available = false;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::servo_unavailable);
    assert(output.disarm_requested);
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    // Losing the execution chain while merely Ready must not move a real leg.
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);

    disabled_input.actuation_available = true;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::servo_unavailable);

    JumpConfig active_config;
    active_config.actuation_enabled = true;
    JumpController action_controller(active_config);
    JumpInput action_input = stableInput();
    output = armAndRequest(action_controller, action_input);
    assert(output.state == JumpState::preload);

    action_input.actuation_available = false;
    output = action_controller.update(action_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::servo_unavailable);
    assert(output.disarm_requested);
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    assert(output.leg_command_valid);
    assert(output.apply_leg_command);
    assert(nearlyEqual(output.common_leg_height_mm, active_config.abort_height_mm));
    assert(nearlyEqual(output.left_leg_height_mm, active_config.abort_height_mm));
    assert(nearlyEqual(output.right_leg_height_mm, active_config.abort_height_mm));
}

void testJumpTimingValidityInterlocksDisabledReadyAndAction()
{
    JumpController disabled_controller;
    JumpInput disabled_input = stableInput();
    disabled_input.timing_valid = false;

    JumpOutput output = repeat(disabled_controller, disabled_input, 25U);
    assert(output.state == JumpState::disabled);
    assert(output.fault == JumpFault::none);
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);

    disabled_input.timing_valid = true;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::ready);
    assert(output.fault == JumpFault::none);

    disabled_input.timing_valid = false;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::control_timing_violation);
    assert(output.disarm_requested);
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    assert(!output.leg_command_valid);
    assert(!output.apply_leg_command);

    disabled_input.timing_valid = true;
    output = disabled_controller.update(disabled_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::control_timing_violation);

    JumpConfig active_config;
    active_config.actuation_enabled = true;
    JumpController action_controller(active_config);
    JumpInput action_input = stableInput();
    output = armAndRequest(action_controller, action_input);
    assert(output.state == JumpState::preload);

    action_input.timing_valid = false;
    output = action_controller.update(action_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::control_timing_violation);
    assert(output.disarm_requested);
    assert(output.freeze_outer_loops);
    assert(output.reset_control_state);
    assert(output.leg_command_valid);
    assert(output.apply_leg_command);
    assert(nearlyEqual(output.common_leg_height_mm, active_config.abort_height_mm));
    assert(nearlyEqual(output.left_leg_height_mm, active_config.abort_height_mm));
    assert(nearlyEqual(output.right_leg_height_mm, active_config.abort_height_mm));
}

void testJumpActuationSettledGatesReadinessWithoutFalseFault()
{
    JumpConfig active_config;
    active_config.actuation_enabled = true;
    JumpController controller(active_config);
    JumpInput input = stableInput();
    input.actuation_settled = false;

    JumpOutput output = controller.update(input);
    assert(output.state == JumpState::ready);
    output = repeat(controller, input, 30U);
    assert(output.state == JumpState::ready);
    assert(output.fault == JumpFault::none);

    input.armed = true;
    output = controller.update(input);
    assert(!output.ready_to_jump);

    input.actuation_settled = true;
    output = repeat(controller, input, 21U);
    assert(output.ready_to_jump);

    input.jump_request = true;
    output = controller.update(input);
    assert(output.state == JumpState::preload);
    input.jump_request = false;
    input.actuation_settled = false;
    output = controller.update(input);
    assert(output.state == JumpState::preload);
    assert(output.fault == JumpFault::none);
}

void testJumpRejectsUnstableAndInvalidInputs()
{
    JumpController unstable_controller;
    JumpInput unstable_input = stableInput();
    JumpOutput output = unstable_controller.update(unstable_input);
    assert(output.state == JumpState::ready);
    output = repeat(unstable_controller, unstable_input, 25U);
    unstable_input.armed = true;
    output = unstable_controller.update(unstable_input);
    assert(output.ready_to_jump);
    unstable_input.roll_degrees = 6.0F;
    unstable_input.jump_request = true;
    output = unstable_controller.update(unstable_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::unstable_request);

    JumpController invalid_controller;
    JumpInput invalid_input = stableInput();
    output = invalid_controller.update(invalid_input);
    assert(output.state == JumpState::ready);
    invalid_input.elapsed_seconds = 0.2F;
    output = invalid_controller.update(invalid_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::invalid_input);

    JumpConfig invalid_config;
    invalid_config.thrust_height_mm = 100.0F;
    JumpController bad_config_controller(invalid_config);
    JumpInput bad_config_input = stableInput();
    output = bad_config_controller.update(bad_config_input);
    assert(output.state == JumpState::fault);
    assert(output.fault == JumpFault::invalid_config);
}

} // namespace

int main()
{
    testControlProfilesContainCompleteBoundedTunings();
    testControlProfileTransitionIsLinearAndRetargetable();
    testJumpDefaultsToDryRunAndRequiresStableArming();
    testJumpNominalPhaseSequenceAndOneShotDisarm();
    testJumpRollTrimIsBoundedAndActuationMustBeExplicit();
    testJumpReadyRequiresPitchWheelAndNeutralControls();
    testJumpActionFaultCommandsBoundedAbortPoseAndStaysLatched();
    testJumpFaultsAreLatchedAndPhasesHaveTimeouts();
    testJumpActuationAvailabilityInterlocksDisabledReadyAndAction();
    testJumpTimingValidityInterlocksDisabledReadyAndAction();
    testJumpActuationSettledGatesReadinessWithoutFalseFault();
    testJumpRejectsUnstableAndInvalidInputs();
    std::cout << "all control safety tests passed\n";
}
