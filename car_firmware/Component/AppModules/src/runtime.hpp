#pragma once

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
    volatile float left_leg_height = 0.0F;
    volatile float right_leg_height = 0.0F;
    volatile float target_height = 44.5F;
    volatile float roll_target = 0.0F;

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

} // namespace wl1::app_modules::detail
