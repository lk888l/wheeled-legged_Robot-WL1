#pragma once
#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <vector>
#include "task.h"
#include "HardwareModule.hpp"
#include "etl/format.h"
#include "etl/string.h"
#include "etl/string_view.h"

template<size_t = 128U>
class LkUart {
public:
    std::vector<std::string> logs;
    etl::string<128U> received;
    template<typename... Args>
    void print(etl::format_string<Args...> format, Args&&... args)
    {
        std::array<char, 256U> text{};
        const auto end = etl::format_to_n(text.data(), text.size(), format, etl::forward<Args>(args)...);
        logs.emplace_back(text.data(), end);
    }
    template<typename Signal>
    bool bindReactor(Signal, TaskHandle_t, uint32_t) { return true; }
    void signal_RxComplete(std::function<void(etl::string<128U>&)> slot) { slot(received); }
};

class NRF24L01P {
public:
    struct Status_t { bool RX_DR{}; bool TX_DS{}; bool MAX_RT{}; };
    std::array<uint8_t, 32U> received{};
    std::vector<std::array<uint8_t, 32U>> sent;
    bool receive_ok{true};
    template<typename Signal>
    bool bindReactor(Signal, TaskHandle_t, uint32_t) { return true; }
    void signal_IRQEvent(std::function<void(Status_t&)> slot)
    {
        Status_t status{true, false, false};
        slot(status);
    }
    bool tryReceive(uint8_t* data)
    {
        if (receive_ok) { std::copy(received.begin(), received.end(), data); }
        return receive_ok;
    }
    bool send(const uint8_t* data, uint8_t size)
    {
        std::array<uint8_t, 32U> payload{};
        std::copy_n(data, std::min<size_t>(size, payload.size()), payload.data());
        sent.push_back(payload);
        return true;
    }
    static void uint8_tostr(etl::string_view& text, uint8_t* data)
    {
        size_t length = 0U;
        while (length < 32U && data[length] != 0U) { ++length; }
        text = {reinterpret_cast<const char*>(data), length};
    }
    static void str_touint8(etl::string_view text, uint8_t* data)
    {
        std::fill_n(data, 32U, 0U);
        std::copy_n(text.data(), std::min<size_t>(text.size(), 32U), data);
    }
};

class MPU6050 {
public:
    struct EulerAngle { double Roll{}, Pitch{}, Yaw{}; };
    bool healthy{true};
    std::vector<TickType_t> samples;
    bool getEulerAngleGyro(EulerAngle& angle, double* gyro)
    {
        assert(fake_rtos::critical_depth == 0); // HAL must run outside critical sections.
        samples.push_back(fake_rtos::now);
        if (!healthy) { return false; }
        angle = {0.0, -48.24, 0.0};
        std::fill_n(gyro, 3U, 0.0);
        return true;
    }
};

class TB6612 {
public:
    int left{}, right{};
    unsigned writes{};
    void forceStop() { left = right = 0; }
    void setAVel_raw(int value) { assert(fake_rtos::critical_depth == 0); left = value; ++writes; }
    void setBVel_raw(int value) { assert(fake_rtos::critical_depth == 0); right = value; ++writes; }
    template<typename T>
    static T clamp(T value, T high, T low) { return std::clamp(value, low, high); }
};

class HallEncoder {
public:
    double getRPM() { return 0.0; }
};

class Servo {
public:
    unsigned stops{};
    float angle{};
    void stop() { ++stops; }
    void setAngle_Smooth(float target, float) { angle = target; }
};

namespace bsp { class BoardHardware; }
namespace fake_board {
inline bsp::BoardHardware* instance{};
inline uint32_t failed_initialization_mask{};
inline std::vector<bsp::HardwareModuleId> initializations;
inline std::vector<TickType_t> initialization_ticks;
inline void (*on_initialize)(bsp::HardwareModuleId){};
inline void reset()
{
    instance = nullptr;
    failed_initialization_mask = 0U;
    initializations.clear();
    initialization_ticks.clear();
    on_initialize = nullptr;
}
}

namespace bsp {
class BoardHardware {
public:
    BoardHardware() { fake_board::instance = this; }
    bool initialize_command_uart() { return initialize(HardwareModuleId::command_uart); }
    bool initialize_imu() { return initialize(HardwareModuleId::imu); }
    bool initialize_left_encoder() { return initialize(HardwareModuleId::left_encoder); }
    bool initialize_right_encoder() { return initialize(HardwareModuleId::right_encoder); }
    bool initialize_wheel_motor() { return initialize(HardwareModuleId::wheel_motor); }
    bool initialize_left_servo() { return initialize(HardwareModuleId::left_servo); }
    bool initialize_right_servo() { return initialize(HardwareModuleId::right_servo); }
    bool initialize_radio() { return initialize(HardwareModuleId::radio); }
    bool pressed{false};
    bool status_led{false};
    bool button_pressed() const { return pressed; }
    void set_status_led(bool on) { status_led = on; }
    LkUart<>& command_uart() { return uart_; }
    NRF24L01P& radio() { return radio_; }
    MPU6050& imu() { return imu_; }
    HallEncoder& left_encoder() { return left_encoder_; }
    HallEncoder& right_encoder() { return right_encoder_; }
    TB6612& wheel_motor() { return wheel_motor_; }
    Servo& left_servo() { return left_servo_; }
    Servo& right_servo() { return right_servo_; }
    unsigned safe_stops{};
    void force_safe_outputs()
    {
        ++safe_stops;
        wheel_motor_.forceStop();
        left_servo_.stop();
        right_servo_.stop();
    }
private:
    bool initialize(HardwareModuleId id)
    {
        fake_board::initializations.push_back(id);
        fake_board::initialization_ticks.push_back(fake_rtos::now);
        if (fake_board::on_initialize != nullptr) { fake_board::on_initialize(id); }
        return (fake_board::failed_initialization_mask & (uint32_t{1} << module_id(id))) == 0U;
    }
    LkUart<> uart_;
    NRF24L01P radio_;
    MPU6050 imu_;
    HallEncoder left_encoder_, right_encoder_;
    TB6612 wheel_motor_;
    Servo left_servo_, right_servo_;
};
}
