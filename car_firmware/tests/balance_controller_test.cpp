#include "CtrlAlgorithm/BalanceController.hpp"
#include "test_check.hpp"
#include <cmath>

namespace {
void near(float actual, float expected, float tolerance = 0.0001F)
{
    CHECK(std::abs(actual - expected) < tolerance);
}
}

int main()
{
    using namespace balance_control;
    Options options;
    CHECK(options.gyro_damping && options.convergence);
    AngleController controller;
    // Body rates for +1 rad/s Euler pitch at 30 degree roll (yaw rate zero).
    near(pitchRateDegrees(30.0F, std::sqrt(3.0) / 2.0, -0.5), 57.2957795F);
    near(pitchRateDegrees(30.0F, 0.5, std::sqrt(3.0) / 2.0), 0.0F);
    near(pitchRateDegrees(0.0F, -1.0, 2.0), -57.2957795F);

    // At the equilibrium angle, moving forward still requires full braking.
    auto output = controller.update(0, 0, 100, 0.01F, 75, 0, 60, options);
    near(output.pwm, -60);
    near(output.terms.p, 0);
    near(output.terms.d, -60);
    // Timing jitter must not multiply the gyro damping gain.
    near(controller.update(0, 0, 100, 0.02F, 75, 0, 60, options).pwm, -60);
    near(controller.update(0, 0, -100, 0.01F, 75, 0, 60, options).pwm, 60);
    // Changing the target or compensated angle must not create derivative kick.
    near(controller.update(2, 0, 0, 0.01F, 75, 0, 60, options).terms.d, 0);
    near(controller.update(2, 3, 0, 0.01F, 75, 0, 60, options).terms.d, 0);
    options.gyro_damping = false;
    near(controller.update(2, 4, 0, 0.01F, 75, 0, 60, options).terms.d, -60);
    controller.reset();
    near(controller.update(2, 4, 0, 0.01F, 75, 0, 60, options).terms.d, 0);

    options = {};
    near(proportionalScale(options, 0, 0), 0.85F);
    near(proportionalScale(options, 1, 0), 0.925F);
    near(proportionalScale(options, -1, 0), 0.925F);
    near(proportionalScale(options, 2, 0), 1);
    near(proportionalScale(options, 0, -20), 1);
    output = controller.update(0, 0, 100, 0.01F, 75, 0, 60, options);
    near(output.pwm, -60); // Scheduling never gates damping at a zero crossing.
    near(output.effective_kp, 75);
    near(controller.update(0, 1, 0, 0.01F, 100, 0, 0, options).pwm, -92.5F);

    controller.reset();
    near(controller.update(1, 0, 0, 0.01F, 0, 1, 0, options).terms.i, 1);
    near(controller.update(4, 0, 0, 0.01F, 0, 1, 0, options).terms.i, 1);
    near(controller.update(1, 0, 25, 0.01F, 0, 1, 0, options).terms.i, 1);
    near(controller.update(1, 0, 0, 0.01F, 0, 1, 0, options).terms.i, 2);
    // Account for the PWM headroom already used by differential steering.
    controller.reset();
    options.near_kp_ratio = 1;
    for (unsigned i = 0; i < 100; ++i) {
        output = controller.update(1, 0, 0, 0.01F, 600, 5, 0, options, 500);
        near(output.pwm, 600); // Retain P/D authority; only stop integral windup.
        near(output.terms.i, 0);
    }
    near(controller.update(0, 0, 0, 0.01F, 600, 5, 0, options, 500).pwm, 0);

    options = {};
    options.rate_filter_hz = 20;
    controller.reset();
    controller.update(0, 0, 0, 0.01F, 0, 0, 60, options);
    output = controller.update(0, 0, 100, 0.01F, 0, 0, 60, options);
    CHECK(output.filtered_pitch_rate_dps > 0 && output.filtered_pitch_rate_dps < 100);
    const auto twice = controller.update(0, 0, 100, 0.01F, 0, 0, 60, options);
    controller.reset();
    controller.update(0, 0, 0, 0.01F, 0, 0, 60, options);
    const auto longer = controller.update(0, 0, 100, 0.02F, 0, 0, 60, options);
    near(twice.filtered_pitch_rate_dps, longer.filtered_pitch_rate_dps);
    controller.reset();
    near(controller.update(0, 0, -100, 0.01F, 0, 0, 60, options).pwm, 60);
    options.rate_filter_hz = 0;
    near(controller.update(0, 0, 100, 0.01F, 0, 0, 60, options).pwm, -60);
}
