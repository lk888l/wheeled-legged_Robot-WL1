#include <cassert>
#include <cmath>
#include <iostream>

#include "LQR.hpp"
#include "LegKinematics.hpp"
#include "PID.hpp"

namespace {

bool nearly_equal(double left, double right, double tolerance = 1.0e-5)
{
    return std::abs(left - right) <= tolerance;
}

void test_pid_derivative_state_is_deterministic()
{
    PID controller(0.0F, 0.0F, 1.0F, -100.0F, 100.0F, -10.0F, 10.0F);
    assert(nearly_equal(controller.update(0.0F, 10.0F), -10.0F));
    assert(nearly_equal(controller.update(0.0F, 10.0F), 0.0F));
}

void test_pid_limits_and_reset()
{
    PID positional(0.0F, 1.0F, 0.0F, -10.0F, 10.0F, -2.0F, 2.0F);
    assert(nearly_equal(positional.update(10.0F, 0.0F), 2.0F));
    assert(nearly_equal(positional.update(10.0F, 0.0F), 2.0F));

    PID incremental(1.0F, 0.0F, 0.0F, -10.0F, 10.0F, -2.0F, 2.0F);
    assert(nearly_equal(incremental.updateIncremental(2.0F, 0.0F), 2.0F));
    incremental.reset();
    assert(nearly_equal(incremental.updateIncremental(1.0F, 0.0F), 1.0F));
}

void test_lqr_calculation()
{
    double gains[4]{1.0, 2.0, 3.0, 4.0};
    LQR controller(gains);
    assert(nearly_equal(controller.Calculate_LQR(1.0, 2.0, 3.0, 4.0), -30.0));
}

void test_leg_inverse_matches_forward_solution()
{
    for (const float target_height : {44.5F, 61.5F, 78.5F})
    {
        float result_x = 0.0F;
        const float angle = LegKinematics::getMotorAngleForHeight(target_height, &result_x);
        const auto point = LegKinematics::forwardKinematics(angle);
        assert(angle >= 0.0F && angle <= 80.0F);
        assert(std::isfinite(result_x));
        assert(std::abs(point.y - target_height) < 0.05F);
    }
}

} // namespace

int main()
{
    test_pid_derivative_state_is_deterministic();
    test_pid_limits_and_reset();
    test_lqr_calculation();
    test_leg_inverse_matches_forward_solution();
    std::cout << "all control algorithm tests passed\n";
}
