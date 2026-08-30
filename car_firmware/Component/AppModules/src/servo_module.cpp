#include "app_modules.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "LegKinematics.hpp"
#include "Servo.hpp"
#include "tim.h"

#include "rtos_task_module.hpp"
#include "runtime.hpp"

namespace wl1::app_modules {
namespace {

constexpr float kMinimumLegHeight = 44.5F;
constexpr float kMaximumLegHeight = 78.5F;

float clamp_leg_height(float height)
{
    if (height > kMaximumLegHeight)
    {
        return kMaximumLegHeight;
    }
    if (height < kMinimumLegHeight)
    {
        return kMinimumLegHeight;
    }
    return height;
}

class ServoModule final : public detail::RtosTaskModule
{
public:
    ServoModule()
        : RtosTaskModule("servo_control",
                         "ServoControl",
                         256,
                         28,
                         detail::runtime().servo_task)
    {
    }

private:
    void run() override
    {
        auto& runtime = detail::runtime();
        auto& control = runtime.control;
        auto& uart = runtime.uart;
        Servo left_servo(&htim9,
                         TIM_CHANNEL_1,
                         Servo::PhysicalToPulse(0.0F),
                         Servo::PhysicalToPulse(180.0F),
                         180.0);
        Servo right_servo(&htim9,
                          TIM_CHANNEL_2,
                          Servo::PhysicalToPulse(169.0F),
                          Servo::PhysicalToPulse(11.0F),
                          180.0);

        const bool left_servo_ready = left_servo.Init();
        const bool right_servo_ready = right_servo.Init();
        if (!left_servo_ready || !right_servo_ready)
        {
            left_servo.stop();
            right_servo.stop();
            taskENTER_CRITICAL();
            control.servo_ready = false;
            control.servo_command_complete = false;
            taskEXIT_CRITICAL();
            uart.print("Servo PWM init failed\n");
            while (true)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        left_servo.setLimit(0, 50);
        right_servo.setLimit(0, 50);
        taskENTER_CRITICAL();
        control.servo_ready = true;
        control.servo_command_complete = false;
        taskEXIT_CRITICAL();

        while (true)
        {
            if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0)
            {
                continue;
            }

            taskENTER_CRITICAL();
            const float left_height = clamp_leg_height(control.left_leg_height);
            const float right_height = clamp_leg_height(control.right_leg_height);
            const std::uint32_t target_sequence = control.servo_target_sequence;
            control.left_leg_height = left_height;
            control.right_leg_height = right_height;
            taskEXIT_CRITICAL();

            float left_x = 0.0F;
            float right_x = 0.0F;
            const float left_degrees = LegKinematics::getMotorAngleForHeight(
                left_height, &left_x);
            const float right_degrees = LegKinematics::getMotorAngleForHeight(
                right_height, &right_x);
            left_servo.setAngle_Smooth(left_degrees - 10.0F, 1000.0F);
            right_servo.setAngle_Smooth(right_degrees - 10.0F, 1000.0F);
            const bool command_complete = left_servo.isCommandComplete()
                && right_servo.isCommandComplete();
            taskENTER_CRITICAL();
            control.servo_consumed_sequence = target_sequence;
            control.servo_command_complete = command_complete;
            control.servo_heartbeat_tick = xTaskGetTickCount();
            taskEXIT_CRITICAL();
        }
    }
};

} // namespace

app::AppModule& servo_module()
{
    static ServoModule module;
    return module;
}

} // namespace wl1::app_modules
