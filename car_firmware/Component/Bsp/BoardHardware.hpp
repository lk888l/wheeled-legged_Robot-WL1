#pragma once

#include <cstdint>

#include "HardwareModule.hpp"
#include "HallEncoder.h"
#include "LkUart.hpp"
#include "MPU6050.h"
#include "NRF24L01P.hpp"
#include "Servo.hpp"
#include "TB6612.h"

namespace bsp {

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

    [[nodiscard]] bool initialize_command_uart();
    [[nodiscard]] bool initialize_imu();
    [[nodiscard]] bool initialize_left_encoder();
    [[nodiscard]] bool initialize_right_encoder();
    [[nodiscard]] bool initialize_wheel_motor();
    [[nodiscard]] bool initialize_left_servo();
    [[nodiscard]] bool initialize_right_servo();
    [[nodiscard]] bool initialize_radio();

    void force_safe_outputs();
    [[nodiscard]] bool button_pressed() const;
    void set_status_led(bool on);

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

} // namespace bsp
