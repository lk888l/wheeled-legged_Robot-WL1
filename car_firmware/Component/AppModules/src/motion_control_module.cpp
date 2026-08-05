#include "app_modules.hpp"

#include <cmath>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "HallEncoder.h"
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
constexpr std::uint8_t kVelocityDivider = 5;

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
                    {MPU6050::GyroRange_t::G1000,
                     MPU6050::AccRange_t::A4,
                     static_cast<std::uint16_t>(MPU6050::ms_toHZ(kControlPeriod)),
                     {0, 0, 0}});
        MPU6050::EulerAngle angles;
        double gyro[3]{};
        HallEncoder left_encoder(&htim2, HallEncoder::InitConfig_t{7, 50, 4, 50});
        HallEncoder right_encoder(&htim3, HallEncoder::InitConfig_t{7, 50, 4, 50});
        TB6612 wheels = create_wheel_driver();

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
        TickType_t last_wake_time = xTaskGetTickCount();

        while (true)
        {
            // Blocking while inside a FreeRTOS critical section prevents the tick and PendSV
            // handlers from running. Keep the periodic wait outside all critical sections.
            vTaskDelayUntil(&last_wake_time, kControlPeriod);

            const float average_height =
                (control.left_leg_height + control.left_leg_height) / 2.0F;
            control.angle_bias =
                (0.01026F * average_height * average_height) - (1.258F * average_height)
                + 48.24F;
            control.angle_kp = (0.3F * average_height) + 56.9F;

            imu.getEulerAngleGyro(angles, gyro);
            control.euler_angles[0] = static_cast<float>(angles.Roll);
            control.euler_angles[1] = static_cast<float>(angles.Pitch);
            control.euler_angles[2] = static_cast<float>(angles.Yaw);
            if (control.show_imu_data)
            {
                uart.print("{:07.3f},{:07.3f},{:07.3f}\n", angles.Roll, angles.Pitch, angles.Yaw);
            }

            ++velocity_loop_count;
            if (velocity_loop_count >= kVelocityDivider)
            {
                velocity_loop_count = 0;
                const double left_rpm = left_encoder.getRPM();
                const double right_rpm = right_encoder.getRPM();
                const double average_rpm = (left_rpm + right_rpm) / 2.0;
                differential_rpm = static_cast<float>(left_rpm - right_rpm);

                velocity_pid.setTunings(
                    control.velocity_kp, control.velocity_ki, control.velocity_kd);
                differential_pid.setTunings(control.differential_kp,
                                             control.differential_ki,
                                             control.differential_kd);
                angle_target = velocity_pid.update(
                    control.velocity_target, static_cast<float>(average_rpm));
                differential_pwm = differential_pid.update(
                    control.differential_target, differential_rpm);

                if (control.show_motor_rpm)
                {
                    uart.print("A: {:07.3f}\tB: {:07.3f}\n", left_rpm, right_rpm);
                }

                roll_pid.setTunings(control.roll_kp, control.roll_ki, 0.0F);
                const float roll_error = control.roll_target - static_cast<float>(angles.Roll);
                if ((last_roll_target > 0.0F && control.roll_target < 0.0F)
                    || (last_roll_target < 0.0F && control.roll_target > 0.0F))
                {
                    roll_pid.reset();
                }
                last_roll_target = control.roll_target;

                float height_adjustment =
                    roll_pid.updateIncremental(control.roll_target, static_cast<float>(angles.Roll));
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

                control.left_leg_height = control.target_height - height_adjustment;
                control.right_leg_height = control.target_height + height_adjustment;
                if (runtime.servo_task != nullptr)
                {
                    xTaskNotifyGive(runtime.servo_task);
                }
            }

            angle_pid.setTunings(control.angle_kp, control.angle_ki, control.angle_kd);
            const float even_pwm = angle_pid.update(
                angle_target, static_cast<float>(angles.Pitch) + control.angle_bias);
            int left_pwm = static_cast<int>(std::round(even_pwm + differential_pwm));
            int right_pwm = static_cast<int>(std::round(even_pwm - differential_pwm));
            left_pwm = TB6612::clamp(left_pwm, 1000, -1000);
            right_pwm = TB6612::clamp(right_pwm, 1000, -1000);
            wheels.setAVel_raw(static_cast<std::int16_t>(left_pwm));
            wheels.setBVel_raw(static_cast<std::int16_t>(right_pwm));
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
