#pragma once

#include "AppTask.hpp"
#include "Button.hpp"

namespace app::task_config {

inline constexpr AppTaskConfig heartbeat{TaskId::heartbeat, "Heartbeat", 192U, 1U};
inline constexpr AppTaskConfig command{TaskId::command_service, "CommandService", 2000U, 28U};
inline constexpr AppTaskConfig servo{TaskId::servo_control, "ServoControl", 256U, 28U};
inline constexpr AppTaskConfig motion{TaskId::motion_control, "MotionControl", 2500U, 29U};
inline constexpr AppTaskConfig button{TaskId::button, "ButtonA0", 128U, 1U};

inline constexpr TickType_t motion_period = pdMS_TO_TICKS(10U);
inline constexpr TickType_t button_period = pdMS_TO_TICKS(5U);
inline constexpr Button::Timing button_timing{
    pdMS_TO_TICKS(20U), pdMS_TO_TICKS(300U), pdMS_TO_TICKS(1000U)};

inline constexpr uint32_t required_task_mask =
    (1UL << static_cast<uint8_t>(TaskId::heartbeat)) |
    (1UL << static_cast<uint8_t>(TaskId::command_service)) |
    (1UL << static_cast<uint8_t>(TaskId::servo_control)) |
    (1UL << static_cast<uint8_t>(TaskId::motion_control));

static_assert(sizeof(TickType_t) == sizeof(uint32_t), "Button clock requires 32-bit ticks");
static_assert(button_period > 0U && motion_period > 0U);
static_assert(button_timing.debounce >= button_period);
static_assert(button_timing.double_click > button_timing.debounce);
static_assert(button_timing.long_press > button_timing.double_click);
static_assert(button.priority < command.priority && button.priority < servo.priority);
static_assert(command.priority < motion.priority && servo.priority < motion.priority);
static_assert(button.priority < configTIMER_TASK_PRIORITY);
static_assert(motion.priority < configMAX_PRIORITIES);

} // namespace app::task_config
