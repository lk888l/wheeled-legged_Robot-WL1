#pragma once

#include <cstdint>

namespace wl1::control {

enum class JumpState : std::uint8_t
{
    disabled,
    ready,
    preload,
    thrust,
    flight,
    landing,
    recover,
    fault,
};

enum class JumpFault : std::uint8_t
{
    none,
    invalid_config,
    invalid_input,
    link_lost,
    imu_invalid,
    servo_unavailable,
    control_timing_violation,
    excessive_roll,
    excessive_pitch,
    excessive_angular_rate,
    excessive_wheel_speed,
    acceleration_out_of_range,
    controls_not_neutral,
    request_not_armed,
    unstable_request,
    disarmed_during_action,
    phase_timeout,
};

struct JumpConfig
{
    // Deliberately false by default: the state machine and telemetry run, but
    // apply_leg_command remains false until a hardware-tested build opts in.
    bool actuation_enabled = false;

    float minimum_leg_height_mm = 44.5F;
    float maximum_leg_height_mm = 78.5F;
    float preload_height_mm = 44.5F;
    float thrust_height_mm = 72.0F;
    float flight_height_mm = 60.0F;
    float landing_height_mm = 52.0F;
    // Used only after a fault from an action state. A fault while merely Ready
    // must not cause an otherwise idle leg to move.
    float abort_height_mm = 52.0F;

    float ready_roll_limit_degrees = 5.0F;
    float ready_pitch_error_limit_degrees = 3.0F;
    float ready_gyro_limit_degrees_per_second = 25.0F;
    float ready_wheel_speed_limit_rpm = 5.0F;
    float ready_acceleration_tolerance_g = 0.15F;
    float ready_stable_time_seconds = 0.20F;

    float active_roll_abort_degrees = 20.0F;
    float active_pitch_error_abort_degrees = 20.0F;
    float active_gyro_abort_degrees_per_second = 350.0F;
    float active_wheel_speed_abort_rpm = 150.0F;
    float maximum_measurable_acceleration_g = 3.8F;

    float launch_acceleration_g = 1.15F;
    float freefall_acceleration_g = 0.55F;
    float landing_impact_acceleration_g = 1.35F;
    float event_debounce_seconds = 0.02F;

    float preload_hold_seconds = 0.25F;
    float preload_settle_seconds = 0.05F;
    float preload_timeout_seconds = 0.60F;
    float thrust_minimum_seconds = 0.06F;
    float thrust_timeout_seconds = 0.25F;
    float flight_minimum_seconds = 0.04F;
    float flight_timeout_seconds = 0.80F;
    float landing_minimum_seconds = 0.08F;
    float landing_settle_seconds = 0.08F;
    float landing_timeout_seconds = 0.45F;
    float recover_motion_seconds = 0.40F;
    float recover_settle_seconds = 0.10F;
    float recover_timeout_seconds = 0.90F;

    // Roll feedback can only trim the left/right height split. It cannot close
    // the loop around the physical servo position or correct pitch.
    float roll_trim_kp_mm_per_degree = 0.15F;
    float roll_trim_kd_mm_per_degree_per_second = 0.01F;
    float maximum_roll_trim_mm = 2.0F;
};

struct JumpInput
{
    float elapsed_seconds = 0.01F;
    bool controller_enabled = false;
    bool armed = false;
    bool jump_request = false;
    bool imu_valid = false;
    bool link_fresh = false;
    // `actuation_available` reports that the servo task/PWM execution chain is
    // alive. `actuation_settled` is the weaker readiness condition that the
    // latest software target has been consumed and its smooth ramp completed.
    bool actuation_available = true;
    bool actuation_settled = true;
    bool timing_valid = true;

    float roll_degrees = 0.0F;
    // Signed balance-loop pitch error, in degrees. The integration should use
    // the same bias/target convention as the wheel balance controller.
    float pitch_error_degrees = 0.0F;
    float roll_rate_degrees_per_second = 0.0F;
    float gyro_norm_degrees_per_second = 0.0F;
    // max(abs(left_rpm), abs(right_rpm)); an average would miss a spin in place.
    float wheel_speed_norm_rpm = 0.0F;
    float acceleration_norm_g = 1.0F;
    bool controls_neutral = true;
    float requested_leg_height_mm = 44.5F;
};

struct JumpOutput
{
    JumpState state = JumpState::disabled;
    JumpFault fault = JumpFault::none;
    bool dry_run = true;
    bool ready_to_jump = false;
    bool action_active = false;

    // A trajectory is still reported during dry-run for logging and tests.
    bool leg_command_valid = false;
    bool apply_leg_command = false;
    float common_leg_height_mm = 44.5F;
    float left_leg_height_mm = 44.5F;
    float right_leg_height_mm = 44.5F;

    bool freeze_outer_loops = false;
    bool reset_control_state = false;
    bool disarm_requested = false;
    float state_elapsed_seconds = 0.0F;
};

/**
 * IMU-supervised, open-loop jump phase controller.
 *
 * This class owns no hardware and performs no allocation. Without servo
 * position feedback it can only supervise a time-based command trajectory and
 * detect gross motion phases from IMU data. A fault is latched until the local
 * controller enable is removed.
 */
class JumpController
{
public:
    explicit JumpController(JumpConfig config = {}) noexcept;

    [[nodiscard]] JumpOutput update(const JumpInput& input) noexcept;
    void reset(bool current_request_level = false) noexcept;

    [[nodiscard]] JumpState state() const noexcept;
    [[nodiscard]] JumpFault fault() const noexcept;
    [[nodiscard]] const JumpConfig& config() const noexcept;

private:
    [[nodiscard]] bool configValid() const noexcept;
    [[nodiscard]] bool inputFinite(const JumpInput& input) const noexcept;
    [[nodiscard]] bool readyObservation(const JumpInput& input) const noexcept;
    [[nodiscard]] bool actionState() const noexcept;

    void transitionTo(JumpState state, bool reset_control_state = false) noexcept;
    void enterFault(JumpFault fault) noexcept;
    [[nodiscard]] JumpOutput takeOutput(const JumpInput& input) noexcept;

    JumpConfig config_;
    JumpState state_ = JumpState::disabled;
    JumpFault fault_ = JumpFault::none;
    float state_elapsed_seconds_ = 0.0F;
    float stable_elapsed_seconds_ = 0.0F;
    float launch_elapsed_seconds_ = 0.0F;
    float freefall_elapsed_seconds_ = 0.0F;
    float impact_elapsed_seconds_ = 0.0F;
    float saved_leg_height_mm_ = 44.5F;
    bool launch_seen_ = false;
    bool last_request_level_ = false;
    bool disarm_required_ = false;
    bool reset_control_requested_ = false;
    bool abort_pose_required_ = false;
};

} // namespace wl1::control
