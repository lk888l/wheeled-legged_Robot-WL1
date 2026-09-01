#pragma once

#include <array>
#include <cstdint>

#include "InitializationManager.hpp"
#include "HallEncoder.h"
#include "LkUart.hpp"
#include "MPU6050.h"
#include "NRF24L01P.hpp"
#include "Servo.hpp"
#include "TB6612.h"

namespace bsp {

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

/**
 * Board-level hardware facade.
 *
 * UserApp owns control policy; this object owns configured device drivers and
 * is the only layer that knows which STM32 HAL handles and GPIO pins implement
 * those devices on WL1.
 */
class BoardHardware {
public:
    BoardHardware();

    BoardHardware(const BoardHardware&) = delete;
    BoardHardware& operator=(const BoardHardware&) = delete;

    bool initialize_command_uart();
    bool initialize_imu();
    bool initialize_left_encoder();
    bool initialize_right_encoder();
    bool initialize_wheel_motor();
    bool initialize_left_servo();
    bool initialize_right_servo();
    bool initialize_radio();

    void force_safe_outputs();

    LkUart<>& command_uart() { return command_uart_; }
    MPU6050& imu() { return imu_; }
    HallEncoder& left_encoder() { return left_encoder_; }
    HallEncoder& right_encoder() { return right_encoder_; }
    TB6612& wheel_motor() { return wheel_motor_; }
    Servo& left_servo() { return left_servo_; }
    Servo& right_servo() { return right_servo_; }
    NRF24L01P& radio() { return radio_; }

private:
    LkUart<> command_uart_;
    MPU6050 imu_;
    HallEncoder left_encoder_;
    HallEncoder right_encoder_;
    TB6612 wheel_motor_;
    Servo left_servo_;
    Servo right_servo_;
    NRF24L01P radio_;
};

class HardwareFactory {
public:
    static constexpr size_t kModuleCount =
        static_cast<size_t>(HardwareModuleId::count);
    using InitializationPlan = std::array<app::InitializationStep, kModuleCount>;

    [[nodiscard]] static InitializationPlan create_initialization_plan(
        BoardHardware& hardware);
};

} // namespace bsp
