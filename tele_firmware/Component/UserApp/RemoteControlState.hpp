#pragma once

namespace RemoteControlLimits {
inline constexpr float speed_minimum = -100.0F;
inline constexpr float speed_maximum = 100.0F;
inline constexpr float turn_minimum = -100.0F;
inline constexpr float turn_maximum = 100.0F;
inline constexpr float leg_minimum_mm = 44.5F;
inline constexpr float leg_maximum_mm = 78.5F;
inline constexpr float roll_minimum_degrees = -18.0F;
inline constexpr float roll_maximum_degrees = 18.0F;
} // namespace RemoteControlLimits

struct RemoteControlSnapshot {
    float speed = 0.0F;
    float turn = 0.0F;
    float leg_height_mm = RemoteControlLimits::leg_minimum_mm;
    float roll_degrees = 0.0F;
    bool leg_locked = false;
    bool roll_locked = false;
};

/**
 * Owns the live joystick inputs and the command values sent to the car.
 * All methods are safe to call from different FreeRTOS tasks.
 */
class RemoteControlState {
public:
    void updateFromJoysticks(
        float speed, float turn, float legHeightMm, float rollDegrees) noexcept;
    [[nodiscard]] bool toggleLegLock() noexcept;
    [[nodiscard]] bool toggleRollLock() noexcept;
    [[nodiscard]] RemoteControlSnapshot snapshot() const noexcept;

private:
    float speed_ = 0.0F;
    float turn_ = 0.0F;
    float live_leg_height_mm_ = RemoteControlLimits::leg_minimum_mm;
    float live_roll_degrees_ = 0.0F;
    float command_leg_height_mm_ = RemoteControlLimits::leg_minimum_mm;
    float command_roll_degrees_ = 0.0F;
    bool leg_locked_ = false;
    bool roll_locked_ = false;
};
