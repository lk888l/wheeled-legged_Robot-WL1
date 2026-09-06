#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "CtrlAlgorithm/BalanceStartupGate.hpp"
#include "CtrlAlgorithm/PID.hpp"
#include "TB6612.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

static void require(bool condition, const char* scenario)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", scenario);
        std::exit(EXIT_FAILURE);
    }
}

static void arm(BalanceStartupGate& gate)
{
    for (unsigned i = 1; i < BalanceStartupGate::stable_samples_required; ++i) {
        require(!gate.update(true, 0, 0, 0, 0, 0), "wait for 500 ms of stable neutral input");
    }
    require(gate.update(true, 0, 0, 0, 0, 0), "arm on the fiftieth stable sample");
}

int main()
{
    BalanceStartupGate gate;
    for (unsigned i = 0; i < 1000; ++i) {
        require(!gate.update(true, -35.0F + 12.6F, 7.0F, 0, 0, 0),
                "observed bench pose must not start the balance controller");
    }
    arm(gate);
    require(gate.update(true, 10, 18, 30, 50, 20), "normal motion targets are accepted after arming");
    require(!gate.update(true, 31, 0, 0, 0, 0), "excessive pitch disarms immediately");
    arm(gate);
    require(!gate.update(true, 0, -31, 0, 0, 0), "excessive roll disarms immediately");
    arm(gate);
    require(!gate.update(false, 0, 0, 0, 0, 0), "IMU read failure disarms immediately");
    require(!gate.update(true, std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0),
            "non-finite attitude cannot arm");
    for (unsigned i = 0; i < 100; ++i) {
        require(!gate.update(true, 0, 0, 0, 1, 0), "moving speed target blocks startup");
        require(!gate.update(true, 0, 0, 0, 0, 1), "turn command blocks startup");
        require(!gate.update(true, 0, 0, 21, 0, 0), "moving body blocks startup");
    }
    for (unsigned i = 0; i < 49; ++i) gate.update(true, 0, 0, 0, 0, 0);
    require(!gate.update(true, 9, 0, 0, 0, 0), "unstable pose interrupts the ready interval");
    arm(gate);

    // Readiness follows the calibrated balance angle, including at other leg heights.
    gate.reset();
    const float current_bias = BalanceCompensation::pitchBias(13.6F, 61.5F);
    for (unsigned i = 0; i < 50; ++i) {
        gate.update(true, -10.70252F + current_bias, 0, 0, 0, 0);
    }
    require(gate.update(true, -10.70252F + current_bias, 0, 0, 0, 0),
            "calibrated baseline remains compatible with automatic startup");

    PID pid(70, 0, 60, -1000, 1000, -100, 100);
    require(pid.update(0, 1) == -70, "first measurement must not produce a derivative impulse");
    require(pid.update(0, 2) == -200, "subsequent derivative calculation is preserved");
    pid.reset();
    require(pid.update(0, 4) == -280, "rearming does not differentiate against a reset zero");
    require(pid.update(0, 4) == -280, "stationary measurement keeps its proportional response");

    TIM_TypeDef timer{};
    TIM_HandleTypeDef handle{&timer};
    GPIO_TypeDef gpio{};
    TB6612 motor({&handle, TIM_CHANNEL_1, TIM_CHANNEL_2, &gpio, 1, &gpio, 2, &gpio, 4, &gpio, 8});
    motor.Init();
    motor.setA_DeadZone(50);
    motor.setB_DeadZone(50);
    for (int command : {250, -250, 1, -1}) {
        motor.setAVel_raw(static_cast<std::int16_t>(command));
        motor.setBVel_raw(static_cast<std::int16_t>(command));
        const auto expected = static_cast<std::uint32_t>(std::abs(command) < 50 ? 50 : std::abs(command));
        require(timer.CCR[0] == expected && timer.CCR[1] == expected, "nonzero motor drive retains its dead zone");
        motor.setAVel_raw(0);
        motor.setBVel_raw(0);
        require(timer.CCR[0] == 0 && timer.CCR[1] == 0, "zero command stays zero after either direction");
    }
    std::puts("PASS: startup/recovery gate, PID first sample and motor zero PWM");
}
