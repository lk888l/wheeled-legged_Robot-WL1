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

void test_pid_nominal_dt_preserves_legacy_behavior()
{
    PID legacy(1.0F, 0.5F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID timed(1.0F, 0.5F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);

    const float legacy_first = legacy.update(3.0F, 1.0F);
    const float timed_first = timed.update(3.0F, 1.0F, 0.01F, 0.01F);
    assert(nearly_equal(legacy_first, 1.0F));
    assert(nearly_equal(timed_first, legacy_first));

    const float legacy_second = legacy.update(3.0F, 1.5F);
    const float timed_second = timed.update(3.0F, 1.5F, 0.01F, 0.01F);
    assert(nearly_equal(legacy_second, 2.25F));
    assert(nearly_equal(timed_second, legacy_second));

    const float legacy_third = legacy.update(2.0F, 1.0F);
    const float timed_third = timed.update(2.0F, 1.0F, 0.01F, 0.01F);
    assert(nearly_equal(legacy_third, 4.25F));
    assert(nearly_equal(timed_third, legacy_third));

    PID legacy_incremental(1.0F, 0.5F, 0.25F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID timed_incremental(1.0F, 0.5F, 0.25F, -100.0F, 100.0F, -100.0F, 100.0F);
    assert(nearly_equal(legacy_incremental.updateIncremental(2.0F, 0.0F), 3.5F));
    assert(nearly_equal(timed_incremental.updateIncremental(2.0F, 0.0F, 0.05F, 0.05F),
                        3.5F));
    assert(nearly_equal(legacy_incremental.updateIncremental(2.0F, 0.5F), 3.125F));
    assert(nearly_equal(timed_incremental.updateIncremental(2.0F, 0.5F, 0.05F, 0.05F),
                        3.125F));
    assert(nearly_equal(legacy_incremental.updateIncremental(1.0F, 0.5F), 2.25F));
    assert(nearly_equal(timed_incremental.updateIncremental(1.0F, 0.5F, 0.05F, 0.05F),
                        2.25F));
}

void test_pid_dt_normalizes_integral_by_wall_time()
{
    PID positional_fine(0.0F, 1.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID positional_coarse(0.0F, 1.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    float fine_output = 0.0F;
    float coarse_output = 0.0F;
    for (int sample = 0; sample < 10; ++sample)
    {
        fine_output = positional_fine.update(1.0F, 0.0F, 0.01F, 0.01F);
    }
    for (int sample = 0; sample < 5; ++sample)
    {
        coarse_output = positional_coarse.update(1.0F, 0.0F, 0.02F, 0.01F);
    }
    assert(nearly_equal(fine_output, 10.0F));
    assert(nearly_equal(coarse_output, fine_output));

    PID incremental_fine(0.0F, 1.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID incremental_coarse(0.0F, 1.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    fine_output = 0.0F;
    coarse_output = 0.0F;
    for (int sample = 0; sample < 10; ++sample)
    {
        fine_output = incremental_fine.updateIncremental(1.0F, 0.0F, 0.01F, 0.01F);
    }
    for (int sample = 0; sample < 5; ++sample)
    {
        coarse_output = incremental_coarse.updateIncremental(1.0F, 0.0F, 0.02F, 0.01F);
    }
    assert(nearly_equal(fine_output, 10.0F));
    assert(nearly_equal(coarse_output, fine_output));
}

void test_pid_dt_normalizes_constant_physical_slope()
{
    PID positional_fine(0.0F, 0.0F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID positional_irregular(0.0F, 0.0F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    positional_fine.prime(0.0F, 0.0F);
    positional_irregular.prime(0.0F, 0.0F);
    float fine_output = positional_fine.update(0.0F, 1.0F, 0.01F, 0.01F);
    fine_output = positional_fine.update(0.0F, 2.0F, 0.01F, 0.01F);
    fine_output = positional_fine.update(0.0F, 3.0F, 0.01F, 0.01F);
    float irregular_output = positional_irregular.update(0.0F, 1.0F, 0.01F, 0.01F);
    irregular_output = positional_irregular.update(0.0F, 3.0F, 0.02F, 0.01F);
    assert(nearly_equal(fine_output, -2.0F));
    assert(nearly_equal(irregular_output, fine_output));

    // Kp keeps the legacy incremental controller active; Kd should represent
    // the same physical error slope after either 10+10+10 ms or 10+20 ms.
    PID incremental_fine(1.0F, 0.0F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    PID incremental_irregular(1.0F, 0.0F, 2.0F, -100.0F, 100.0F, -100.0F, 100.0F);
    incremental_fine.prime(0.0F, 0.0F);
    incremental_irregular.prime(0.0F, 0.0F);
    fine_output = incremental_fine.updateIncremental(0.0F, 1.0F, 0.01F, 0.01F);
    fine_output = incremental_fine.updateIncremental(0.0F, 2.0F, 0.01F, 0.01F);
    fine_output = incremental_fine.updateIncremental(0.0F, 3.0F, 0.01F, 0.01F);
    irregular_output = incremental_irregular.updateIncremental(
        0.0F, 1.0F, 0.01F, 0.01F);
    irregular_output = incremental_irregular.updateIncremental(
        0.0F, 3.0F, 0.02F, 0.01F);
    assert(nearly_equal(fine_output, -5.0F));
    assert(nearly_equal(irregular_output, fine_output));
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
    test_pid_nominal_dt_preserves_legacy_behavior();
    test_pid_dt_normalizes_integral_by_wall_time();
    test_pid_dt_normalizes_constant_physical_slope();
    test_lqr_calculation();
    test_leg_inverse_matches_forward_solution();
    std::cout << "all control algorithm tests passed\n";
}
