#pragma once

#include <cstdint>

namespace bsp {

// Stable diagnostic bit positions. Append new modules before count; do not
// reorder existing IDs because UART status and ST-Link use these bitmaps.
enum class HardwareModuleId : uint8_t {
    command_uart = 0,
    imu,
    left_encoder,
    right_encoder,
    wheel_motor,
    left_servo,
    right_servo,
    radio,
    count,
};

constexpr uint8_t module_id(HardwareModuleId id)
{
    return static_cast<uint8_t>(id);
}

inline constexpr uint8_t kHardwareModuleCount = module_id(HardwareModuleId::count);
static_assert(kHardwareModuleCount > 0U && kHardwareModuleCount <= 32U);
inline constexpr uint32_t kRequiredHardwareMask =
    UINT32_MAX >> (32U - kHardwareModuleCount);

constexpr const char* module_name(HardwareModuleId id)
{
    switch (id) {
    case HardwareModuleId::command_uart: return "command-uart";
    case HardwareModuleId::imu: return "imu-mpu6050";
    case HardwareModuleId::left_encoder: return "left-encoder";
    case HardwareModuleId::right_encoder: return "right-encoder";
    case HardwareModuleId::wheel_motor: return "wheel-motor";
    case HardwareModuleId::left_servo: return "left-servo";
    case HardwareModuleId::right_servo: return "right-servo";
    case HardwareModuleId::radio: return "radio-nrf24";
    case HardwareModuleId::count: break;
    }
    return "unknown";
}

} // namespace bsp
