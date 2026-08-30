#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "LkUart.hpp"
#include "NRF24L01P.hpp"

namespace wl1::app_modules::detail {

struct ControlState
{
    volatile bool show_imu_data = false;
    volatile bool show_motor_rpm = false;

    volatile float angle_kp = 70.0F;
    volatile float angle_ki = 0.0F;
    volatile float angle_kd = 60.0F;
    volatile float angle_bias = 12.6F;
    // angle_kp is the effective value used by the control loop. A manual
    // override is kept separately so height compensation cannot erase it.
    volatile float angle_kp_override = 70.0F;
    volatile bool angle_kp_override_enabled = false;
    volatile float angle_bias_override = 12.6F;
    volatile bool angle_bias_override_enabled = false;

    volatile float velocity_kp = 0.05F;
    volatile float velocity_ki = 0.008F;
    volatile float velocity_kd = 0.0F;
    volatile float velocity_target = 0.0F;

    volatile float differential_kp = 2.0F;
    volatile float differential_ki = 0.001F;
    volatile float differential_kd = 0.0F;
    volatile float differential_target = 0.0F;

    volatile float roll_kp = 0.0F;
    volatile float roll_ki = -0.4F;

    volatile float euler_angles[3]{};
    volatile float left_leg_height = 44.5F;
    volatile float right_leg_height = 44.5F;
    volatile float target_height = 44.5F;
    volatile float roll_target = 0.0F;

    volatile std::uint8_t requested_pid_level = 1;
    volatile std::uint8_t active_pid_level = 1;

    volatile bool remote_command_valid = false;
    volatile TickType_t last_remote_command_tick = 0;
    // Separate radio freshness from local UART control. A serial test stream
    // must not make experimental jump actuation believe the RF link is alive.
    volatile bool wireless_command_valid = false;
    volatile TickType_t last_wireless_command_tick = 0;

    volatile bool jump_armed = false;
    volatile bool jump_request = false;
    volatile bool jump_request_bit_latched = false;
    volatile bool jump_arm_bit_latched = false;
    volatile bool jump_fault_clear_request = false;
    volatile std::uint8_t jump_state = 0;

    // Servo execution-chain health. These are software acknowledgements only;
    // the hardware has no shaft-position feedback.
    volatile bool servo_ready = false;
    volatile bool servo_command_complete = false;
    volatile std::uint32_t servo_target_sequence = 0;
    volatile std::uint32_t servo_consumed_sequence = 0;
    volatile TickType_t servo_heartbeat_tick = 0;

    volatile bool imu_valid = false;
    volatile std::uint32_t imu_read_failures = 0;
    volatile std::uint32_t imu_consecutive_failures = 0;
    volatile std::uint32_t imu_saturation_events = 0;
    volatile std::uint32_t imu_recovery_samples = 0;
    volatile std::uint32_t imu_accel_rejected_streak = 0;
    volatile float acceleration_norm_g = 0.0F;
    volatile float angular_velocity_dps[3]{};

    volatile bool nrf_print_enabled = false;
    const volatile float* nrf_print_values[4];

    ControlState();
};

struct Runtime
{
    Runtime();

    LkUart<> uart;
    NRF24L01P radio;
    ControlState control;
    TaskHandle_t communication_task = nullptr;
    TaskHandle_t servo_task = nullptr;
    TaskHandle_t motion_task = nullptr;
};

Runtime& runtime();
// ISR-safe: returns nullptr until runtime() has completed construction.
[[nodiscard]] Runtime* runtime_if_ready() noexcept;

} // namespace wl1::app_modules::detail
