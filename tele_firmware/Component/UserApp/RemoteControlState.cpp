#include "RemoteControlState.hpp"

#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

namespace {
float clamp(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}
} // namespace

void RemoteControlState::updateFromJoysticks(
    float speed, float turn, float legHeightMm, float rollDegrees)
{
    const float limited_speed = clamp(
        speed, RemoteControlLimits::speed_minimum, RemoteControlLimits::speed_maximum);
    const float limited_turn = clamp(
        turn, RemoteControlLimits::turn_minimum, RemoteControlLimits::turn_maximum);
    const float limited_leg = clamp(
        legHeightMm, RemoteControlLimits::leg_minimum_mm, RemoteControlLimits::leg_maximum_mm);
    const float limited_roll = clamp(
        rollDegrees,
        RemoteControlLimits::roll_minimum_degrees,
        RemoteControlLimits::roll_maximum_degrees);

    taskENTER_CRITICAL();
    speed_ = limited_speed;
    turn_ = limited_turn;
    live_leg_height_mm_ = limited_leg;
    live_roll_degrees_ = limited_roll;
    if (!leg_locked_) {
        command_leg_height_mm_ = live_leg_height_mm_;
    }
    if (!roll_locked_) {
        command_roll_degrees_ = live_roll_degrees_;
    }
    taskEXIT_CRITICAL();
}

bool RemoteControlState::toggleLegLock()
{
    taskENTER_CRITICAL();
    leg_locked_ = !leg_locked_;
    command_leg_height_mm_ = live_leg_height_mm_;
    const bool locked = leg_locked_;
    taskEXIT_CRITICAL();
    return locked;
}

bool RemoteControlState::toggleRollLock()
{
    taskENTER_CRITICAL();
    roll_locked_ = !roll_locked_;
    command_roll_degrees_ = live_roll_degrees_;
    const bool locked = roll_locked_;
    taskEXIT_CRITICAL();
    return locked;
}

RemoteControlSnapshot RemoteControlState::snapshot() const
{
    taskENTER_CRITICAL();
    const RemoteControlSnapshot result{
        .speed = speed_,
        .turn = turn_,
        .leg_height_mm = command_leg_height_mm_,
        .roll_degrees = command_roll_degrees_,
        .leg_locked = leg_locked_,
        .roll_locked = roll_locked_,
    };
    taskEXIT_CRITICAL();
    return result;
}
