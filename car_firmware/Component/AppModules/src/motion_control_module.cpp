#include "app_modules.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "HallEncoder.h"
#include "ControlProfile.hpp"
#include "JumpController.hpp"
#include "LQR.hpp"
#include "MPU6050.h"
#include "PID.hpp"
#include "TB6612.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"

#include "rtos_task_module.hpp"
#include "runtime.hpp"

namespace wl1::app_modules {
namespace {

constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(10);
constexpr TickType_t kMaximumControlInterval = pdMS_TO_TICKS(30);
constexpr std::uint8_t kVelocityDivider = 5;
constexpr float kControlPeriodSeconds = 0.010F;
constexpr float kVelocityPeriodSeconds =
    kControlPeriodSeconds * static_cast<float>(kVelocityDivider);
constexpr TickType_t kRemoteFailsafeTimeout = pdMS_TO_TICKS(250);
constexpr TickType_t kServoHeartbeatTimeout = pdMS_TO_TICKS(200);
constexpr float kRadiansToDegrees = 57.2957795F;
constexpr bool kJumpActuationEnabled = WL1_ENABLE_EXPERIMENTAL_JUMP != 0;
constexpr bool kRequireUprightStartup = WL1_REQUIRE_UPRIGHT_STARTUP != 0;
constexpr std::uint32_t kImmediateImuRecoverySamples = 1U;
constexpr std::uint32_t kTransientImuRecoverySamples = 3U;
constexpr std::uint32_t kFusionImuRecoverySamples = 20U;
constexpr std::uint32_t kMaximumGyroOnlySamples = 50U;
// The guarded mode re-enables only near the established balance point.
// angle_pid.reset() primes derivative history on the enabling sample, so an
// 8-degree proportional error remains below full PWM while normal balancing
// retains full recovery authority. The default direct-start mode deliberately
// bypasses these pose/motion gates and waits only for a currently trusted IMU
// gravity sample.
constexpr float kRecoveryPitchErrorLimitDegrees = 8.0F;
constexpr float kRecoveryRollLimitDegrees = 10.0F;
constexpr float kRecoveryGyroLimitDegreesPerSecond = 40.0F;
constexpr float kRecoveryWheelSpeedLimitRpm = 5.0F;
constexpr float kJumpLegRateMmPerSecond = 400.0F;
constexpr float kJumpAbortTargetEpsilonMm = 0.05F;
constexpr TickType_t kJumpAbortSettleTicks = pdMS_TO_TICKS(200);
constexpr float kMinimumLegHeightMm = 44.5F;
constexpr float kMaximumLegHeightMm = 78.5F;
constexpr float kAngleKpTuningRatePerSecond = 80.0F;
constexpr float kAngleKiTuningRatePerSecond = 1.0F;
constexpr float kAngleKdTuningRatePerSecond = 70.0F;
constexpr float kAngleBiasTuningRateDegreesPerSecond = 5.0F;
constexpr float kVelocityKpTuningRatePerSecond = 0.15F;
constexpr float kVelocityKiTuningRatePerSecond = 0.03F;
constexpr float kVelocityKdTuningRatePerSecond = 0.10F;
constexpr float kDifferentialKpTuningRatePerSecond = 5.0F;
constexpr float kDifferentialKiTuningRatePerSecond = 0.01F;
constexpr float kDifferentialKdTuningRatePerSecond = 1.0F;
constexpr float kRollKpTuningRatePerSecond = 2.0F;
constexpr float kRollKiTuningRatePerSecond = 1.0F;

struct ControlSnapshot
{
    bool show_imu_data = false;
    bool show_motor_rpm = false;
    float angle_ki = 0.0F;
    float angle_kd = 0.0F;
    float angle_kp_override = 0.0F;
    bool angle_kp_override_enabled = false;
    float angle_bias_override = 0.0F;
    bool angle_bias_override_enabled = false;
    float velocity_kp = 0.0F;
    float velocity_ki = 0.0F;
    float velocity_kd = 0.0F;
    float differential_kp = 0.0F;
    float differential_ki = 0.0F;
    float differential_kd = 0.0F;
    float roll_kp = 0.0F;
    float roll_ki = 0.0F;
    float velocity_target = 0.0F;
    float differential_target = 0.0F;
    float roll_target = 0.0F;
    float target_height = 44.5F;
    float left_leg_height = 44.5F;
    float right_leg_height = 44.5F;
    std::uint8_t requested_pid_level = 1;
    bool wireless_fresh = false;
    bool jump_armed = false;
    bool jump_request = false;
    bool jump_fault_clear_request = false;
    bool servo_ready = false;
    bool servo_command_complete = false;
    std::uint32_t servo_target_sequence = 0;
    std::uint32_t servo_consumed_sequence = 0;
    TickType_t servo_heartbeat_tick = 0;
};

ControlSnapshot captureControlState(detail::ControlState& control, TickType_t now)
{
    ControlSnapshot result;
    taskENTER_CRITICAL();
    if (control.remote_command_valid
        && (now - control.last_remote_command_tick) > kRemoteFailsafeTimeout)
    {
        control.velocity_target = 0.0F;
        control.differential_target = 0.0F;
        control.roll_target = 0.0F;
        control.remote_command_valid = false;
        control.jump_armed = false;
        control.jump_request = false;
        control.jump_request_bit_latched = false;
    }
    if (control.wireless_command_valid
        && (now - control.last_wireless_command_tick) > kRemoteFailsafeTimeout)
    {
        control.wireless_command_valid = false;
        control.jump_armed = false;
        control.jump_request = false;
        control.jump_request_bit_latched = false;
    }

    result.show_imu_data = control.show_imu_data;
    result.show_motor_rpm = control.show_motor_rpm;
    result.angle_ki = control.angle_ki;
    result.angle_kd = control.angle_kd;
    result.angle_kp_override = control.angle_kp_override;
    result.angle_kp_override_enabled = control.angle_kp_override_enabled;
    result.angle_bias_override = control.angle_bias_override;
    result.angle_bias_override_enabled = control.angle_bias_override_enabled;
    result.velocity_kp = control.velocity_kp;
    result.velocity_ki = control.velocity_ki;
    result.velocity_kd = control.velocity_kd;
    result.differential_kp = control.differential_kp;
    result.differential_ki = control.differential_ki;
    result.differential_kd = control.differential_kd;
    result.roll_kp = control.roll_kp;
    result.roll_ki = control.roll_ki;
    result.velocity_target = control.velocity_target;
    result.differential_target = control.differential_target;
    result.roll_target = control.roll_target;
    result.target_height = control.target_height;
    result.left_leg_height = control.left_leg_height;
    result.right_leg_height = control.right_leg_height;
    result.requested_pid_level = control.requested_pid_level;
    result.wireless_fresh = control.wireless_command_valid
        && (now - control.last_wireless_command_tick) <= kRemoteFailsafeTimeout;
    result.jump_armed = control.jump_armed;
    result.jump_request = control.jump_request;
    result.jump_fault_clear_request = control.jump_fault_clear_request;
    result.servo_ready = control.servo_ready;
    result.servo_command_complete = control.servo_command_complete;
    result.servo_target_sequence = control.servo_target_sequence;
    result.servo_consumed_sequence = control.servo_consumed_sequence;
    result.servo_heartbeat_tick = control.servo_heartbeat_tick;
    control.jump_request = false;
    control.jump_fault_clear_request = false;
    taskEXIT_CRITICAL();
    return result;
}

float scaleNormalGain(float normal_gain, float profile_gain, float profile_normal_gain)
{
    if (std::fabs(profile_normal_gain) <= 1.0e-6F)
    {
        return normal_gain;
    }
    return normal_gain * (profile_gain / profile_normal_gain);
}

float symmetricClamp(float value, float magnitude)
{
    return std::clamp(value, -magnitude, magnitude);
}

float slewToward(float current,
                 float target,
                 float maximum_rate_per_second,
                 float elapsed_seconds)
{
    if (!std::isfinite(target))
    {
        return current;
    }
    if (!std::isfinite(current))
    {
        return target;
    }
    const float maximum_step = std::max(0.0F, maximum_rate_per_second)
        * std::max(0.0F, elapsed_seconds);
    const float remaining = target - current;
    if (std::fabs(remaining) <= maximum_step)
    {
        // Snap to the requested value at the end of a ramp. Apart from avoiding
        // a one-ulp tail, this gives the jump readiness gate an unambiguous
        // indication that every live-tuning transition has finished.
        return target;
    }
    return current + std::clamp(remaining, -maximum_step, maximum_step);
}

bool isJumpActionState(wl1::control::JumpState state)
{
    return state == wl1::control::JumpState::preload
        || state == wl1::control::JumpState::thrust
        || state == wl1::control::JumpState::flight
        || state == wl1::control::JumpState::landing
        || state == wl1::control::JumpState::recover;
}

TB6612 create_wheel_driver()
{
    return TB6612(TB6612::InitConfig_t{
        .Htim = &htim1,
        .AChannel = TIM_CHANNEL_1,
        .BChannel = TIM_CHANNEL_2,
        .A1GPIO_Port = AIN1_GPIO_Port,
        .A1GPIO_Pin = AIN1_Pin,
        .A2GPIO_Port = AIN2_GPIO_Port,
        .A2GPIO_Pin = AIN2_Pin,
        .B1GPIO_Port = BIN1_GPIO_Port,
        .B1GPIO_Pin = BIN1_Pin,
        .B2GPIO_Port = BIN2_GPIO_Port,
        .B2GPIO_Pin = BIN2_Pin,
    });
}

[[maybe_unused]] void run_legacy_lqr_control()
{
    auto& runtime = detail::runtime();
    auto& uart = runtime.uart;
    auto& control = runtime.control;

    MPU6050 imu(&hi2c1,
                {MPU6050::GyroRange_t::G1000,
                 MPU6050::AccRange_t::A4,
                 static_cast<std::uint16_t>(MPU6050::ms_toHZ(2)),
                 {0, 0, 0}});
    MPU6050::EulerAngle angles;
    double gyro[3]{};
    double gains[4]{-4.569790, -4.472503, 0.0, 0.0};
    LQR controller(gains);
    HallEncoder left_encoder(&htim2, HallEncoder::InitConfig_t{7, 150, 4, kControlPeriod});
    HallEncoder right_encoder(&htim3, HallEncoder::InitConfig_t{7, 150, 4, kControlPeriod});
    TB6612 wheels = create_wheel_driver();

    wheels.Init();
    wheels.setDirection_Cfg(static_cast<std::uint8_t>(TB6612::OutPort::A),
                            TB6612::Direction::Negative);
    wheels.setA_DeadZone(0);
    wheels.setB_DeadZone(0);

    imu.setGyroOffset(2.5, 0.7, 0.9);
    if (imu.Init())
    {
        uart.print("MPU: success\n");
    }
    else
    {
        uart.print("MPU: fail\n");
    }
    left_encoder.clearCounter();
    right_encoder.clearCounter();

    double left_position = 0.0;
    double right_position = 0.0;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&last_wake_time, kControlPeriod);

        imu.getEulerAngleGyro(angles, gyro);
        if (control.show_imu_data)
        {
            uart.print("{:07.3f},{:07.3f},{:07.3f}\n", angles.Roll, angles.Pitch, angles.Yaw);
        }

        const double lqr_angle = MPU6050::DegTorad(angles.Pitch + 20.0);
        const double lqr_gyro = -MPU6050::DegTorad(gyro[1]);
        const double left_rpm = left_encoder.getRPM();
        const double right_rpm = right_encoder.getRPM();
        const double left_velocity = HallEncoder::Rpm_ToMS(LQR::WheelRadius, -left_rpm) / 60;
        const double right_velocity = HallEncoder::Rpm_ToMS(LQR::WheelRadius, -right_rpm) / 60;

        left_position = -HallEncoder::Rpm_ToMS(
            LQR::WheelRadius,
            HallEncoder::Cnt_toTurnNum(left_encoder, left_encoder.getAccumCnt()));
        right_position = -HallEncoder::Rpm_ToMS(
            LQR::WheelRadius,
            HallEncoder::Cnt_toTurnNum(right_encoder, right_encoder.getAccumCnt()));
        left_position = TB6612::clamp(left_position, 5.0, -5.0);
        right_position = TB6612::clamp(right_position, 5.0, -5.0);

        int left_pwm = static_cast<int>(std::round(
            controller.Calculate_LQR(lqr_angle, lqr_gyro, left_position, left_velocity) * 1200));
        int right_pwm = static_cast<int>(std::round(
            controller.Calculate_LQR(lqr_angle, lqr_gyro, right_position, right_velocity)
            * 1200));
        left_pwm = TB6612::clamp(left_pwm, 1000, -1000);
        right_pwm = TB6612::clamp(right_pwm, 1000, -1000);

        if (control.show_motor_rpm)
        {
            uart.print("A: {:07.3f}\tB: {:07.3f}\n", left_rpm, right_rpm);
        }

        std::uint32_t notified_value = 0;
        if (xTaskNotifyWait(0, 0xFFFFFFFFU, &notified_value, 0) == pdTRUE)
        {
            uart.print("Motor output:{}\t{}\n",
                       notified_value >> 16U,
                       notified_value & 0xFFFFU);
            wheels.setBVel_raw(static_cast<std::int16_t>(notified_value >> 16U));
            wheels.setAVel_raw(static_cast<std::int16_t>(notified_value & 0xFFFFU));
        }
    }
}

class MotionControlModule final : public detail::RtosTaskModule
{
public:
    MotionControlModule()
        : RtosTaskModule("motion_control",
                         "MotionControl",
                         2500,
                         29,
                         detail::runtime().motion_task)
    {
    }

private:
    void run() override
    {
        auto& runtime = detail::runtime();
        auto& control = runtime.control;
        auto& uart = runtime.uart;

        PID angle_pid(70.0F, 0.0F, 51.0F, -1000.0F, 1000.0F, -100.0F, 100.0F);
        PID velocity_pid(0.04F, 0.006F, 0.0F, -10.0F, 10.0F, -100.0F, 100.0F);
        PID differential_pid(0.0F, 0.0F, 0.0F, -500.0F, 500.0F, -100.0F, 100.0F);
        PID roll_pid(0.0F, 0.0F, 0.0F, -78.0F, 78.0F, -100.0F, 100.0F);

        MPU6050 imu(&hi2c1,
                    {MPU6050::GyroRange_t::G2000,
                     MPU6050::AccRange_t::A8,
                     static_cast<std::uint16_t>(MPU6050::ms_toHZ(kControlPeriod)),
                     {0, 0, 0}});
        MPU6050::Sample imu_sample;
        HallEncoder left_encoder(&htim2, HallEncoder::InitConfig_t{7, 50, 4, 50});
        HallEncoder right_encoder(&htim3, HallEncoder::InitConfig_t{7, 50, 4, 50});
        TB6612 wheels = create_wheel_driver();

        wl1::control::ControlProfileTransition profile_transition(
            wl1::control::ControlStrength::normal);
        wl1::control::JumpConfig jump_config;
        jump_config.actuation_enabled = kJumpActuationEnabled;
        wl1::control::JumpController jump_controller(jump_config);

        wheels.Init();
        wheels.setDirection_Cfg(static_cast<std::uint8_t>(TB6612::OutPort::B),
                                TB6612::Direction::Negative);
        wheels.setA_DeadZone(50);
        wheels.setB_DeadZone(50);

        imu.setGyroOffset(2.5, 0.7, 0.9);
        if (imu.Init())
        {
            uart.print("MPU: success\n");
        }
        else
        {
            uart.print("MPU: fail\n");
            // Attitude feedback is mandatory for balancing. Keep both PWM channels at
            // zero and yield so lower-priority diagnostics remain available.
            wheels.setA_DeadZone(0);
            wheels.setB_DeadZone(0);
            wheels.setAVel_raw(0);
            wheels.setBVel_raw(0);
            while (true)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        left_encoder.clearCounter();
        right_encoder.clearCounter();

        std::uint8_t velocity_loop_count = 0;
        float differential_rpm = 0.0F;
        float angle_target = 0.0F;
        float differential_pwm = 0.0F;
        float last_roll_target = 0.0F;
        std::uint32_t consecutive_imu_failures = 0;
        std::uint32_t imu_recovery_samples = 0;
        std::uint32_t required_imu_recovery_samples = kRequireUprightStartup
            ? kFusionImuRecoverySamples
            : kImmediateImuRecoverySamples;
        std::uint32_t accel_rejected_streak = 0;
        bool imu_control_enabled = false;
        bool wheel_speed_observed_during_recovery = false;
        bool jump_fault_clear_pending = false;
        TickType_t jump_abort_reached_since = 0;
        double left_rpm = 0.0;
        double right_rpm = 0.0;
        double average_rpm = 0.0;
        double left_rpm_accumulator = 0.0;
        double right_rpm_accumulator = 0.0;
        double encoder_interval_accumulator_ms = 0.0;
        float velocity_window_seconds = kVelocityPeriodSeconds;
        float wheel_speed_norm_rpm = 0.0F;
        float published_left_height = kMinimumLegHeightMm;
        float published_right_height = kMinimumLegHeightMm;
        float smoothed_angle_kp = 70.0F;
        float smoothed_angle_ki = 0.0F;
        float smoothed_angle_kd = 60.0F;
        float smoothed_angle_bias = 12.6F;
        float smoothed_velocity_kp = 0.05F;
        float smoothed_velocity_ki = 0.008F;
        float smoothed_velocity_kd = 0.0F;
        float smoothed_differential_kp = 2.0F;
        float smoothed_differential_ki = 0.001F;
        float smoothed_differential_kd = 0.0F;
        float smoothed_roll_kp = 0.0F;
        float smoothed_roll_ki = -0.4F;
        int last_left_pwm = 0;
        int last_right_pwm = 0;
        TickType_t last_wake_time = xTaskGetTickCount();
        TickType_t last_control_tick = last_wake_time;
        TickType_t last_encoder_sample_tick = last_wake_time;

        const auto stop_wheels_immediately = [&]() {
            wheels.setAVel_raw(0);
            wheels.setBVel_raw(0);
            last_left_pwm = 0;
            last_right_pwm = 0;
        };
        const auto publish_leg_targets = [&](float left_target,
                                             float right_target,
                                             float maximum_rate_mm_per_second,
                                             float elapsed_seconds) {
            left_target = std::clamp(
                left_target, kMinimumLegHeightMm, kMaximumLegHeightMm);
            right_target = std::clamp(
                right_target, kMinimumLegHeightMm, kMaximumLegHeightMm);
            const float maximum_step = std::max(0.0F, maximum_rate_mm_per_second)
                * elapsed_seconds;
            published_left_height += std::clamp(
                left_target - published_left_height, -maximum_step, maximum_step);
            published_right_height += std::clamp(
                right_target - published_right_height, -maximum_step, maximum_step);

            taskENTER_CRITICAL();
            const bool target_changed =
                std::fabs(control.left_leg_height - published_left_height) > 1.0e-4F
                || std::fabs(control.right_leg_height - published_right_height) > 1.0e-4F;
            control.left_leg_height = published_left_height;
            control.right_leg_height = published_right_height;
            if (target_changed)
            {
                control.servo_target_sequence = control.servo_target_sequence + 1U;
            }
            taskEXIT_CRITICAL();
            if (runtime.servo_task != nullptr)
            {
                xTaskNotifyGive(runtime.servo_task);
            }
        };

        while (true)
        {
            // Blocking while inside a FreeRTOS critical section prevents the tick and PendSV
            // handlers from running. Keep the periodic wait outside all critical sections.
            vTaskDelayUntil(&last_wake_time, kControlPeriod);

            const TickType_t now = xTaskGetTickCount();
            TickType_t control_elapsed_ticks = now - last_control_tick;
            if (control_elapsed_ticks == 0U)
            {
                // A delayed vTaskDelayUntil() may return immediately while the
                // tick is unchanged. Resynchronise instead of advancing the
                // estimator, PID and jump FSM with invented time.
                last_wake_time = now;
                continue;
            }
            last_control_tick = now;
            if (control_elapsed_ticks >= (2U * kControlPeriod))
            {
                // Do not let vTaskDelayUntil execute a burst of catch-up loops
                // after a debugger stop or long ISR/task stall.
                last_wake_time = now;
            }
            const bool control_timing_valid =
                control_elapsed_ticks > 0U
                && control_elapsed_ticks <= kMaximumControlInterval;
            const float control_elapsed_seconds = std::clamp(
                static_cast<float>(control_elapsed_ticks)
                    * static_cast<float>(portTICK_PERIOD_MS) / 1000.0F,
                0.001F,
                0.10F);
            if (!control_timing_valid && imu_control_enabled)
            {
                imu.resetFusion();
                imu_control_enabled = false;
                imu_recovery_samples = 0;
                required_imu_recovery_samples = kRequireUprightStartup
                    ? kFusionImuRecoverySamples
                    : kImmediateImuRecoverySamples;
                wheel_speed_observed_during_recovery = false;
                accel_rejected_streak = 0;
                angle_pid.reset();
                velocity_pid.reset();
                differential_pid.reset();
                roll_pid.reset();
                angle_target = 0.0F;
                differential_pwm = 0.0F;
                stop_wheels_immediately();
            }
            const ControlSnapshot command = captureControlState(control, now);
            jump_fault_clear_pending = jump_fault_clear_pending
                || command.jump_fault_clear_request;
            const bool servo_execution_alive = command.servo_ready
                && command.servo_heartbeat_tick != 0U
                && (now - command.servo_heartbeat_tick) <= kServoHeartbeatTimeout;
            const bool servo_target_acknowledged =
                command.servo_consumed_sequence == command.servo_target_sequence;
            const bool jump_actuation_available = !kJumpActuationEnabled
                || servo_execution_alive;
            const bool jump_actuation_settled = !kJumpActuationEnabled
                || (servo_execution_alive && servo_target_acknowledged
                    && command.servo_command_complete);

            // Sample wheel safety at the 10 ms control rate, using the measured
            // interval so scheduler jitter cannot scale RPM incorrectly. The
            // outer velocity loop still receives a 50 ms average, preserving its
            // established update rate while jump/recovery gates see fresh motion.
            bool velocity_sample_ready = false;
            TickType_t encoder_elapsed_ticks = now - last_encoder_sample_tick;
            if (encoder_elapsed_ticks == 0U)
            {
                // The zero-time control iteration above is skipped, so this is
                // only a defensive guard. Never fabricate an encoder interval.
                last_wake_time = now;
                continue;
            }
            last_encoder_sample_tick = now;
            const double encoder_elapsed_ms = static_cast<double>(encoder_elapsed_ticks)
                * static_cast<double>(portTICK_PERIOD_MS);
            const double instantaneous_left_rpm =
                left_encoder.getRPM(encoder_elapsed_ms);
            const double instantaneous_right_rpm =
                right_encoder.getRPM(encoder_elapsed_ms);
            wheel_speed_norm_rpm = static_cast<float>(
                std::max(std::fabs(instantaneous_left_rpm),
                         std::fabs(instantaneous_right_rpm)));
            if (!imu_control_enabled)
            {
                wheel_speed_observed_during_recovery = true;
            }
            left_rpm_accumulator += instantaneous_left_rpm * encoder_elapsed_ms;
            right_rpm_accumulator += instantaneous_right_rpm * encoder_elapsed_ms;
            encoder_interval_accumulator_ms += encoder_elapsed_ms;
            ++velocity_loop_count;
            if (velocity_loop_count >= kVelocityDivider)
            {
                const double window_ms = std::max(encoder_interval_accumulator_ms, 1.0);
                left_rpm = left_rpm_accumulator / window_ms;
                right_rpm = right_rpm_accumulator / window_ms;
                velocity_window_seconds = static_cast<float>(window_ms / 1000.0);
                left_rpm_accumulator = 0.0;
                right_rpm_accumulator = 0.0;
                encoder_interval_accumulator_ms = 0.0;
                velocity_loop_count = 0;
                average_rpm = (left_rpm + right_rpm) / 2.0;
                differential_rpm = static_cast<float>(left_rpm - right_rpm);
                velocity_sample_ready = true;
            }
            const auto requested_strength = wl1::control::sanitizeControlStrength(
                static_cast<wl1::control::ControlStrength>(command.requested_pid_level));
            const auto jump_state_before_update = jump_controller.state();
            const bool profile_locked = kJumpActuationEnabled
                && isJumpActionState(jump_state_before_update);
            const bool profile_changed = !profile_locked
                && requested_strength != profile_transition.target();
            if (!profile_locked)
            {
                profile_transition.request(requested_strength);
            }
            const auto& profile = profile_transition.advance(control_elapsed_seconds);
            control.active_pid_level = static_cast<std::uint8_t>(profile_transition.selected());

            // Both legs matter here. The old left+left typo made roll compensation
            // corrupt the pitch bias and Kp calculation.
            const float average_height =
                (command.left_leg_height + command.right_leg_height) / 2.0F;
            const auto automatic_angle_gains =
                wl1::control::angleGainsForHeight(profile, average_height);
            const float target_angle_kp = command.angle_kp_override_enabled
                ? std::clamp(command.angle_kp_override, 0.0F, 200.0F)
                : automatic_angle_gains.kp;
            const float automatic_angle_bias =
                (0.01026F * average_height * average_height)
                - (1.258F * average_height) + 48.24F;
            const float target_angle_bias = command.angle_bias_override_enabled
                ? std::clamp(command.angle_bias_override, 5.0F, 20.0F)
                : automatic_angle_bias;
            const bool raw_tuning_locked = isJumpActionState(jump_state_before_update)
                || jump_state_before_update == wl1::control::JumpState::fault;
            if (!raw_tuning_locked)
            {
                smoothed_angle_kp = slewToward(smoothed_angle_kp,
                                               target_angle_kp,
                                               kAngleKpTuningRatePerSecond,
                                               control_elapsed_seconds);
                smoothed_angle_ki = slewToward(smoothed_angle_ki,
                                               command.angle_ki,
                                               kAngleKiTuningRatePerSecond,
                                               control_elapsed_seconds);
                smoothed_angle_kd = slewToward(smoothed_angle_kd,
                                               command.angle_kd,
                                               kAngleKdTuningRatePerSecond,
                                               control_elapsed_seconds);
                smoothed_angle_bias = slewToward(smoothed_angle_bias,
                                                 target_angle_bias,
                                                 kAngleBiasTuningRateDegreesPerSecond,
                                                 control_elapsed_seconds);
                smoothed_velocity_kp = slewToward(smoothed_velocity_kp,
                                                  command.velocity_kp,
                                                  kVelocityKpTuningRatePerSecond,
                                                  control_elapsed_seconds);
                smoothed_velocity_ki = slewToward(smoothed_velocity_ki,
                                                  command.velocity_ki,
                                                  kVelocityKiTuningRatePerSecond,
                                                  control_elapsed_seconds);
                smoothed_velocity_kd = slewToward(smoothed_velocity_kd,
                                                  command.velocity_kd,
                                                  kVelocityKdTuningRatePerSecond,
                                                  control_elapsed_seconds);
                smoothed_differential_kp = slewToward(smoothed_differential_kp,
                                                      command.differential_kp,
                                                      kDifferentialKpTuningRatePerSecond,
                                                      control_elapsed_seconds);
                smoothed_differential_ki = slewToward(smoothed_differential_ki,
                                                      command.differential_ki,
                                                      kDifferentialKiTuningRatePerSecond,
                                                      control_elapsed_seconds);
                smoothed_differential_kd = slewToward(smoothed_differential_kd,
                                                      command.differential_kd,
                                                      kDifferentialKdTuningRatePerSecond,
                                                      control_elapsed_seconds);
                smoothed_roll_kp = slewToward(smoothed_roll_kp,
                                              command.roll_kp,
                                              kRollKpTuningRatePerSecond,
                                              control_elapsed_seconds);
                smoothed_roll_ki = slewToward(smoothed_roll_ki,
                                              command.roll_ki,
                                              kRollKiTuningRatePerSecond,
                                              control_elapsed_seconds);
            }
            const bool raw_tuning_settled = smoothed_angle_kp == target_angle_kp
                && smoothed_angle_ki == command.angle_ki
                && smoothed_angle_kd == command.angle_kd
                && smoothed_angle_bias == target_angle_bias
                && smoothed_velocity_kp == command.velocity_kp
                && smoothed_velocity_ki == command.velocity_ki
                && smoothed_velocity_kd == command.velocity_kd
                && smoothed_differential_kp == command.differential_kp
                && smoothed_differential_ki == command.differential_ki
                && smoothed_differential_kd == command.differential_kd
                && smoothed_roll_kp == command.roll_kp
                && smoothed_roll_ki == command.roll_ki;
            const float effective_angle_kp = smoothed_angle_kp;
            const float effective_angle_bias = smoothed_angle_bias;
            control.angle_kp = effective_angle_kp;
            control.angle_bias = effective_angle_bias;

            const bool command_controls_neutral =
                std::fabs(command.velocity_target) <= 1.0F
                && std::fabs(command.differential_target) <= 1.0F
                && std::fabs(command.roll_target) <= 1.0F;
            const bool stationary_reacquisition_allowed = !imu_control_enabled
                && wheel_speed_observed_during_recovery
                && wheel_speed_norm_rpm <= kRecoveryWheelSpeedLimitRpm
                && command_controls_neutral
                && !isJumpActionState(jump_controller.state());
            const bool sample_read =
                imu.getSample(imu_sample, stationary_reacquisition_allowed);
            if (sample_read && imu_sample.saturation_flags != 0U)
            {
                control.imu_saturation_events = control.imu_saturation_events + 1U;
            }
            if (!sample_read || !imu_sample.valid)
            {
                const bool control_was_enabled = imu_control_enabled;
                ++consecutive_imu_failures;
                control.imu_consecutive_failures = consecutive_imu_failures;
                control.imu_valid = false;
                control.imu_recovery_samples = 0;
                if (!sample_read)
                {
                    control.imu_read_failures = control.imu_read_failures + 1U;
                }

                constexpr std::uint8_t kGyroSaturationMask =
                    MPU6050::GyroXRawSaturated | MPU6050::GyroYRawSaturated
                    | MPU6050::GyroZRawSaturated;
                const bool gyro_saturated =
                    (imu_sample.saturation_flags & kGyroSaturationMask) != 0U;
                if (gyro_saturated || consecutive_imu_failures == 3U)
                {
                    // Saturation already resets the filter in MPU6050. Three
                    // missed/invalid frames also leave too much unintegrated
                    // time, so discard fusion history before reacquiring gravity.
                    if (!gyro_saturated)
                    {
                        imu.resetFusion();
                    }
                    required_imu_recovery_samples = kRequireUprightStartup
                        ? kFusionImuRecoverySamples
                        : kImmediateImuRecoverySamples;
                }
                else if (consecutive_imu_failures == 1U && control_was_enabled)
                {
                    required_imu_recovery_samples = kRequireUprightStartup
                        ? kTransientImuRecoverySamples
                        : kImmediateImuRecoverySamples;
                }
                imu_control_enabled = false;
                if (control_was_enabled)
                {
                    // Require a new encoder observation after the stop, rather
                    // than trusting the last pre-fault 50 ms sample.
                    wheel_speed_observed_during_recovery = false;
                }
                imu_recovery_samples = 0;
                accel_rejected_streak = 0;
                control.imu_accel_rejected_streak = 0;
                stop_wheels_immediately();
                if (consecutive_imu_failures == 1U)
                {
                    angle_pid.reset();
                    velocity_pid.reset();
                    differential_pid.reset();
                    roll_pid.reset();
                    angle_target = 0.0F;
                    differential_pwm = 0.0F;
                }

                const auto jump_fault_output = jump_controller.update({
                    .elapsed_seconds = control_elapsed_seconds,
                    .controller_enabled = true,
                    .armed = command.jump_armed,
                    .jump_request = command.jump_request,
                    .imu_valid = false,
                    .link_fresh = kJumpActuationEnabled ? command.wireless_fresh : true,
                    .actuation_available = jump_actuation_available,
                    .actuation_settled = jump_actuation_settled,
                    .timing_valid = control_timing_valid,
                    .requested_leg_height_mm = command.target_height,
                });
                control.jump_state = static_cast<std::uint8_t>(jump_fault_output.state);
                if (jump_fault_output.state != wl1::control::JumpState::fault)
                {
                    // A disarm outside Fault is not an acknowledgement for some
                    // future fault.
                    jump_fault_clear_pending = false;
                    jump_abort_reached_since = 0;
                }
                if (jump_fault_output.disarm_requested)
                {
                    taskENTER_CRITICAL();
                    control.jump_armed = false;
                    control.jump_request = false;
                    taskEXIT_CRITICAL();
                }
                if (jump_fault_output.apply_leg_command)
                {
                    publish_leg_targets(jump_fault_output.left_leg_height_mm,
                                        jump_fault_output.right_leg_height_mm,
                                        kJumpLegRateMmPerSecond,
                                        control_elapsed_seconds);
                }
                continue;
            }

            bool recovered_from_imu_failure = false;
            control.euler_angles[0] = static_cast<float>(imu_sample.angle.Roll);
            control.euler_angles[1] = static_cast<float>(imu_sample.angle.Pitch);
            control.euler_angles[2] = static_cast<float>(imu_sample.angle.Yaw);

            float acceleration_norm_g = 0.0F;
            float gyro_norm_degrees_per_second = 0.0F;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const float gyro_dps =
                    static_cast<float>(imu_sample.gyro[axis]) * kRadiansToDegrees;
                control.angular_velocity_dps[axis] = gyro_dps;
                gyro_norm_degrees_per_second += gyro_dps * gyro_dps;
                const float acceleration = static_cast<float>(imu_sample.accel[axis]);
                acceleration_norm_g += acceleration * acceleration;
            }
            gyro_norm_degrees_per_second = std::sqrt(gyro_norm_degrees_per_second);
            acceleration_norm_g = std::sqrt(acceleration_norm_g) / 9.81F;
            control.acceleration_norm_g = acceleration_norm_g;

            if (command.show_imu_data)
            {
                uart.print("{:07.3f},{:07.3f},{:07.3f},a={:05.2f},ok={}\n",
                           imu_sample.angle.Roll,
                           imu_sample.angle.Pitch,
                           imu_sample.angle.Yaw,
                           acceleration_norm_g,
                           imu_sample.accel_trusted ? 1 : 0);
            }

            if (imu_sample.accel_trusted)
            {
                accel_rejected_streak = 0;
            }
            else if (accel_rejected_streak < UINT32_MAX)
            {
                ++accel_rejected_streak;
            }
            control.imu_accel_rejected_streak = accel_rejected_streak;

            const float upright_pitch_error =
                static_cast<float>(imu_sample.angle.Pitch) + effective_angle_bias;
            const bool actual_jump_action = kJumpActuationEnabled
                && isJumpActionState(jump_controller.state());
            if (imu_control_enabled && !actual_jump_action
                && accel_rejected_streak >= kMaximumGyroOnlySamples)
            {
                // Normal balancing must not run indefinitely without a gravity
                // correction. Jump phases have their own bounded timeouts.
                imu_control_enabled = false;
                imu_recovery_samples = 0;
                required_imu_recovery_samples = kRequireUprightStartup
                    ? kFusionImuRecoverySamples
                    : kImmediateImuRecoverySamples;
                wheel_speed_observed_during_recovery = false;
            }

            if (!imu_control_enabled)
            {
                const bool guarded_pose_ready = wheel_speed_observed_during_recovery
                    && wheel_speed_norm_rpm <= kRecoveryWheelSpeedLimitRpm
                    && std::fabs(upright_pitch_error)
                        <= kRecoveryPitchErrorLimitDegrees
                    && std::fabs(static_cast<float>(imu_sample.angle.Roll))
                        <= kRecoveryRollLimitDegrees
                    && gyro_norm_degrees_per_second
                        <= kRecoveryGyroLimitDegreesPerSecond;
                const bool safe_recovery_observation = imu_sample.accel_trusted
                    && (!kRequireUprightStartup || guarded_pose_ready);
                if (safe_recovery_observation)
                {
                    ++imu_recovery_samples;
                }
                else
                {
                    imu_recovery_samples = 0;
                }
                control.imu_recovery_samples = imu_recovery_samples;
                control.imu_valid = false;
                stop_wheels_immediately();

                if (imu_recovery_samples < required_imu_recovery_samples)
                {
                    const auto jump_recovery_output = jump_controller.update({
                        .elapsed_seconds = control_elapsed_seconds,
                        .controller_enabled = true,
                        .armed = command.jump_armed,
                        .jump_request = command.jump_request,
                        .imu_valid = false,
                        .link_fresh = kJumpActuationEnabled
                            ? command.wireless_fresh
                            : true,
                        .actuation_available = jump_actuation_available,
                        .actuation_settled = jump_actuation_settled,
                        .timing_valid = control_timing_valid,
                        .roll_degrees = static_cast<float>(imu_sample.angle.Roll),
                        .pitch_error_degrees = upright_pitch_error,
                        .roll_rate_degrees_per_second = control.angular_velocity_dps[0],
                        .gyro_norm_degrees_per_second = gyro_norm_degrees_per_second,
                        .wheel_speed_norm_rpm = wheel_speed_norm_rpm,
                        .acceleration_norm_g = acceleration_norm_g,
                        .controls_neutral = false,
                        .requested_leg_height_mm = command.target_height,
                    });
                    control.jump_state =
                        static_cast<std::uint8_t>(jump_recovery_output.state);
                    if (jump_recovery_output.state != wl1::control::JumpState::fault)
                    {
                        jump_fault_clear_pending = false;
                        jump_abort_reached_since = 0;
                    }
                    if (jump_recovery_output.disarm_requested)
                    {
                        taskENTER_CRITICAL();
                        control.jump_armed = false;
                        control.jump_request = false;
                        taskEXIT_CRITICAL();
                    }
                    if (jump_recovery_output.apply_leg_command)
                    {
                        publish_leg_targets(jump_recovery_output.left_leg_height_mm,
                                            jump_recovery_output.right_leg_height_mm,
                                            kJumpLegRateMmPerSecond,
                                            control_elapsed_seconds);
                    }
                    continue;
                }

                imu_control_enabled = true;
                recovered_from_imu_failure = true;
                consecutive_imu_failures = 0;
                control.imu_consecutive_failures = 0;
            }
            control.imu_valid = true;

            // Ready must wait until all live-tuning ramps finish. Once an action
            // starts, writes are atomically rejected and the applied values are
            // frozen; automatic height-derived targets may then move with the
            // commanded legs without falsely tripping controls_not_neutral.
            const bool raw_tuning_safe = raw_tuning_locked || raw_tuning_settled;
            const bool controls_neutral = command_controls_neutral
                && !profile_transition.transitioning()
                && raw_tuning_safe;
            const auto jump_output = jump_controller.update({
                .elapsed_seconds = control_elapsed_seconds,
                .controller_enabled = true,
                .armed = command.jump_armed,
                .jump_request = command.jump_request,
                .imu_valid = true,
                .link_fresh = kJumpActuationEnabled ? command.wireless_fresh : true,
                .actuation_available = jump_actuation_available,
                .actuation_settled = jump_actuation_settled,
                .timing_valid = control_timing_valid,
                .roll_degrees = static_cast<float>(imu_sample.angle.Roll),
                .pitch_error_degrees = upright_pitch_error - angle_target,
                .roll_rate_degrees_per_second = control.angular_velocity_dps[0],
                .gyro_norm_degrees_per_second = gyro_norm_degrees_per_second,
                .wheel_speed_norm_rpm = wheel_speed_norm_rpm,
                .acceleration_norm_g = acceleration_norm_g,
                .controls_neutral = controls_neutral,
                .requested_leg_height_mm = command.target_height,
            });
            control.jump_state = static_cast<std::uint8_t>(jump_output.state);
            bool clear_jump_fault_after_output = false;
            if (jump_output.state != wl1::control::JumpState::fault)
            {
                jump_fault_clear_pending = false;
                jump_abort_reached_since = 0;
            }
            else if (jump_fault_clear_pending && !command.jump_armed)
            {
                if (!jump_output.apply_leg_command)
                {
                    // Dry-run or an idle-state fault has no physical abort pose
                    // to finish before acknowledging the fault.
                    clear_jump_fault_after_output = true;
                }
                else
                {
                    const float abort_height = jump_controller.config().abort_height_mm;
                    const bool abort_command_reached =
                        std::fabs(published_left_height - abort_height)
                            <= kJumpAbortTargetEpsilonMm
                        && std::fabs(published_right_height - abort_height)
                            <= kJumpAbortTargetEpsilonMm;
                    const bool servo_abort_acknowledged = jump_actuation_settled;
                    if (abort_command_reached && servo_abort_acknowledged)
                    {
                        if (jump_abort_reached_since == 0U)
                        {
                            jump_abort_reached_since = now;
                        }
                    }
                    else
                    {
                        jump_abort_reached_since = 0;
                    }
                    clear_jump_fault_after_output = jump_abort_reached_since != 0U
                        && (now - jump_abort_reached_since) >= kJumpAbortSettleTicks;
                }
            }
            if (jump_output.disarm_requested)
            {
                taskENTER_CRITICAL();
                control.jump_armed = false;
                control.jump_request = false;
                taskEXIT_CRITICAL();
            }

            angle_pid.setLimits(profile.angle.limits.minimum_output,
                                profile.angle.limits.maximum_output,
                                profile.angle.limits.minimum_integral,
                                profile.angle.limits.maximum_integral);
            if (profile_changed)
            {
                // Retain the integrator across a smooth profile transition while
                // refreshing derivative history at the current operating point.
                angle_pid.prime(
                    angle_target,
                    static_cast<float>(imu_sample.angle.Pitch) + effective_angle_bias);
            }
            if (recovered_from_imu_failure)
            {
                angle_target = 0.0F;
                differential_pwm = 0.0F;
                angle_pid.reset(
                    angle_target,
                    static_cast<float>(imu_sample.angle.Pitch) + effective_angle_bias);
                velocity_pid.reset();
                differential_pid.reset();
                roll_pid.reset();
            }
            if (kJumpActuationEnabled && jump_output.reset_control_state)
            {
                velocity_pid.reset();
                differential_pid.reset();
                roll_pid.reset();
                angle_target = 0.0F;
                differential_pwm = 0.0F;
            }

            if (velocity_sample_ready)
            {
                const auto& normal = wl1::control::kNormalControlProfile;
                velocity_pid.setLimits(profile.velocity.limits.minimum_output,
                                       profile.velocity.limits.maximum_output,
                                       profile.velocity.limits.minimum_integral,
                                       profile.velocity.limits.maximum_integral);
                differential_pid.setLimits(
                    profile.differential.limits.minimum_output,
                    profile.differential.limits.maximum_output,
                    profile.differential.limits.minimum_integral,
                    profile.differential.limits.maximum_integral);
                velocity_pid.setTunings(
                    scaleNormalGain(smoothed_velocity_kp,
                                    profile.velocity.gains.kp,
                                    normal.velocity.gains.kp),
                    scaleNormalGain(smoothed_velocity_ki,
                                    profile.velocity.gains.ki,
                                    normal.velocity.gains.ki),
                    scaleNormalGain(smoothed_velocity_kd,
                                    profile.velocity.gains.kd,
                                    normal.velocity.gains.kd));
                differential_pid.setTunings(
                    scaleNormalGain(smoothed_differential_kp,
                                    profile.differential.gains.kp,
                                    normal.differential.gains.kp),
                    scaleNormalGain(smoothed_differential_ki,
                                    profile.differential.gains.ki,
                                    normal.differential.gains.ki),
                    scaleNormalGain(smoothed_differential_kd,
                                    profile.differential.gains.kd,
                                    normal.differential.gains.kd));

                const bool freeze_outer_loops =
                    kJumpActuationEnabled && jump_output.freeze_outer_loops;
                if (freeze_outer_loops)
                {
                    velocity_pid.reset(0.0F, static_cast<float>(average_rpm));
                    differential_pid.reset(0.0F, differential_rpm);
                    angle_target = 0.0F;
                    differential_pwm = 0.0F;
                }
                else
                {
                    const float velocity_target = symmetricClamp(
                        command.velocity_target,
                        profile.commands.maximum_velocity_target_rpm);
                    const float differential_target = symmetricClamp(
                        command.differential_target,
                        profile.commands.maximum_differential_target_rpm);
                    angle_target = velocity_pid.update(
                        velocity_target,
                        static_cast<float>(average_rpm),
                        velocity_window_seconds,
                        kVelocityPeriodSeconds);
                    angle_target = symmetricClamp(
                        angle_target, profile.commands.maximum_angle_target_degrees);
                    differential_pwm = differential_pid.update(
                        differential_target,
                        differential_rpm,
                        velocity_window_seconds,
                        kVelocityPeriodSeconds);
                    differential_pwm = std::clamp(
                        differential_pwm,
                        profile.differential.limits.minimum_output,
                        profile.differential.limits.maximum_output);
                }

                if (command.show_motor_rpm)
                {
                    uart.print("A: {:07.3f}\tB: {:07.3f}\n", left_rpm, right_rpm);
                }

                roll_pid.setTunings(
                    scaleNormalGain(smoothed_roll_kp,
                                    profile.roll.gains.kp,
                                    normal.roll.gains.kp),
                    scaleNormalGain(smoothed_roll_ki,
                                    profile.roll.gains.ki,
                                    normal.roll.gains.ki),
                    0.0F);
                roll_pid.setLimits(profile.roll.limits.minimum_output,
                                   profile.roll.limits.maximum_output,
                                   profile.roll.limits.minimum_integral,
                                   profile.roll.limits.maximum_integral);
                const float roll_target = symmetricClamp(
                    command.roll_target, profile.commands.maximum_roll_target_degrees);
                const float roll_error =
                    roll_target - static_cast<float>(imu_sample.angle.Roll);
                if ((last_roll_target > 0.0F && roll_target < 0.0F)
                    || (last_roll_target < 0.0F && roll_target > 0.0F))
                {
                    roll_pid.reset(roll_target, static_cast<float>(imu_sample.angle.Roll));
                }
                last_roll_target = roll_target;

                float height_adjustment =
                    roll_pid.updateIncremental(
                        roll_target,
                        static_cast<float>(imu_sample.angle.Roll),
                        velocity_window_seconds,
                        kVelocityPeriodSeconds);
                constexpr float kRollThresholdDegrees = 3.0F;
                constexpr float kGeometricCompensation = 0.5F;
                constexpr float kDegreesToRadians = 0.0174532925F;
                if (roll_error > kRollThresholdDegrees)
                {
                    height_adjustment += kGeometricCompensation * 55.0F
                        * std::sin((roll_error - kRollThresholdDegrees) * kDegreesToRadians);
                }
                else if (roll_error < -kRollThresholdDegrees)
                {
                    height_adjustment += kGeometricCompensation * 55.0F
                        * std::sin((roll_error + kRollThresholdDegrees) * kDegreesToRadians);
                }

                height_adjustment = std::clamp(
                    height_adjustment,
                    profile.roll.limits.minimum_output,
                    profile.roll.limits.maximum_output);
                const float desired_height = std::clamp(
                    command.target_height,
                    profile.commands.minimum_leg_height_mm,
                    profile.commands.maximum_leg_height_mm);
                if (!jump_output.apply_leg_command)
                {
                    publish_leg_targets(
                        desired_height - height_adjustment,
                        desired_height + height_adjustment,
                        profile.commands.maximum_leg_rate_mm_per_second,
                        velocity_window_seconds);
                }
            }

            if (jump_output.apply_leg_command)
            {
                publish_leg_targets(jump_output.left_leg_height_mm,
                                    jump_output.right_leg_height_mm,
                                    kJumpLegRateMmPerSecond,
                                    control_elapsed_seconds);
            }

            angle_pid.setTunings(
                effective_angle_kp,
                smoothed_angle_ki,
                scaleNormalGain(smoothed_angle_kd,
                                profile.angle.gains.kd,
                                wl1::control::kNormalControlProfile.angle.gains.kd));
            const float even_pwm = angle_pid.update(
                angle_target,
                static_cast<float>(imu_sample.angle.Pitch) + effective_angle_bias,
                control_elapsed_seconds,
                kControlPeriodSeconds);
            int left_pwm = static_cast<int>(std::round(even_pwm + differential_pwm));
            int right_pwm = static_cast<int>(std::round(even_pwm - differential_pwm));
            const int maximum_pwm = static_cast<int>(std::round(profile.commands.maximum_motor_pwm));
            left_pwm = TB6612::clamp(left_pwm, maximum_pwm, -maximum_pwm);
            right_pwm = TB6612::clamp(right_pwm, maximum_pwm, -maximum_pwm);
            const int maximum_pwm_step = std::max(
                1,
                static_cast<int>(std::round(
                    profile.commands.maximum_motor_pwm_slew_per_second
                    * control_elapsed_seconds)));
            left_pwm = last_left_pwm + std::clamp(
                left_pwm - last_left_pwm, -maximum_pwm_step, maximum_pwm_step);
            right_pwm = last_right_pwm + std::clamp(
                right_pwm - last_right_pwm, -maximum_pwm_step, maximum_pwm_step);
            wheels.setAVel_raw(static_cast<std::int16_t>(left_pwm));
            wheels.setBVel_raw(static_cast<std::int16_t>(right_pwm));
            last_left_pwm = left_pwm;
            last_right_pwm = right_pwm;
            if (clear_jump_fault_after_output)
            {
                // Reset only after this iteration has published the final abort
                // target. With no servo position feedback this proves completion
                // of the commanded trajectory, not physical shaft position.
                jump_controller.reset(false);
                jump_fault_clear_pending = false;
                jump_abort_reached_since = 0;
                control.jump_state =
                    static_cast<std::uint8_t>(wl1::control::JumpState::disabled);
            }
        }
    }
};

} // namespace

app::AppModule& motion_control_module()
{
    static MotionControlModule module;
    return module;
}

} // namespace wl1::app_modules
