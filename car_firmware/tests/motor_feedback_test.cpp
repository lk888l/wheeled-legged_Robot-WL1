#include <cmath>
#include <limits>
#include "TB6612.h"
#include "HallEncoder.h"
#include "WheelPwm.hpp"
#include "test_check.hpp"

int main()
{
    TIM_TypeDef timer;
    TIM_HandleTypeDef pwm{&timer};
    GPIO_TypeDef pins;
    TB6612 motor({&pwm, TIM_CHANNEL_1, TIM_CHANNEL_2,
                   &pins, 1, &pins, 2, &pins, 4, &pins, 8});
    CHECK(motor.Init());
    motor.setA_DeadZone(75);
    motor.setB_DeadZone(75);
    motor.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B), TB6612::Direction::Negative);
    motor.setAVel_raw(1);
    motor.setBVel_raw(1);
    CHECK(timer.compare[0] == 2 && timer.compare[1] == 2); // Previously 75.
    CHECK(pins.pins == (1 | 8));
    motor.setAVel_raw(-1);
    motor.setBVel_raw(-1);
    CHECK(timer.compare[0] == 2 && timer.compare[1] == 2);
    CHECK(pins.pins == (2 | 4));
    motor.setAVel_raw(0);
    motor.setBVel_raw(0);
    CHECK(timer.compare[0] == 0 && timer.compare[1] == 0);
    motor.setAVel_raw(40);
    motor.setBVel_raw(-40);
    CHECK(timer.compare[0] == 75 && timer.compare[1] == 75);
    motor.setAVel_raw(32767);
    motor.setBVel_raw(-32768);
    CHECK(timer.compare[0] == 1000 && timer.compare[1] == 1000);
    motor.forceStop();
    CHECK(timer.compare[0] == 0 && timer.compare[1] == 0 && pins.pins == 0);

    for (unsigned deadzone : {0U, 20U, 75U, 1000U, 65535U}) {
        int previous = -1000;
        for (int request = -1100; request <= 1100; ++request) {
            const int output = WheelPwm::compensate(request, deadzone);
            CHECK(output >= previous && std::abs(output) <= 1000);
            CHECK(output == -WheelPwm::compensate(-request, deadzone));
            if (deadzone == 0) CHECK(output == std::clamp(request, -1000, 1000));
            previous = output;
        }
        CHECK(WheelPwm::compensate(0, deadzone) == 0);
    }

    for (auto* instance : {TIM2, TIM3}) {
        TIM_HandleTypeDef timer_handle{instance};
        HallEncoder encoder(&timer_handle, {7, 50, 4, 50});
        CHECK(encoder.Init());
        encoder.clearCounter();
        instance->counter = 7;
        CHECK(std::abs(encoder.getRPM() - 6) < 1e-9);
        instance->counter = 21;
        CHECK(std::abs(encoder.getRPM(100) - 6) < 1e-9); // Same speed, delayed sample.
        encoder.clearCounter();
        instance->counter = instance == TIM2 ? 0xfffffffeU : 0xfffeU;
        CHECK(encoder.getCounter() == -2); // Reverse through wrap.
        instance->counter = 2;
        CHECK(encoder.getCounter() == 4); // Forward through wrap.
        CHECK(encoder.getRPM(0) == 0);
        CHECK(encoder.getRPM(std::numeric_limits<float>::quiet_NaN()) == 0);
    }
}
