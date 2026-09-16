#include <cstring>
#include <new>
#include <limits>
#include "CtrlAlgorithm/PID.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp" // Header must be safe to include twice.
#include "test_check.hpp"

int main()
{
    // Different previous stack contents must not change the first motor result.
    for (unsigned char pattern : {0x00, 0x55, 0x7F, 0xFF}) {
        alignas(PID) unsigned char storage[sizeof(PID)];
        std::memset(storage, pattern, sizeof(storage));
        auto* pid = new (storage) PID(1.0F, 0.0F, 2.0F, -100.0F, 100.0F, -10.0F, 10.0F);
        CHECK(pid->update(10.0F, 3.0F) == 7.0F);
        CHECK(pid->update(10.0F, 4.0F) == 4.0F);
        pid->reset();
        CHECK(pid->update(10.0F, 3.0F) == 7.0F);
        pid->~PID();
    }
    PID derivative_only(0, 0, 2, -100, 100, -10, 10);
    CHECK(derivative_only.updateIncremental(1, 0) == 2);
    derivative_only.setTunings(0, 0, 0);
    CHECK(derivative_only.updateIncremental(1, 0) == 0);
    derivative_only.setTunings(1, 0, 0);
    CHECK(derivative_only.updateIncremental(1, 0) == 1);

    // Saturation must not leave an integral tail after the error disappears.
    for (float sign : {-1.0F, 1.0F}) {
        PID saturated(10 * sign, sign, 0, -10, 10, -100, 100);
        for (int i = 0; i < 100; ++i) CHECK(saturated.update(5, 0) == sign * 10);
        CHECK(saturated.update(0, 0) == 0);
        PID integral_only(0, sign, 0, -10, 10, -100, 100);
        CHECK(integral_only.update(20, 0) == sign * 10);
        CHECK(integral_only.update(20, 0) == sign * 10);
        CHECK(integral_only.update(-1, 0) == sign * 9);
    }
    PID unwind(1, 1, 0, -10, 10, -100, 100);
    CHECK(unwind.update(3, 0) == 6);
    CHECK(unwind.update(3, 0) == 9);
    CHECK(unwind.update(-1, 0) == 4); // Existing integral can unwind.
    CHECK(unwind.update(1, 0, 1, false) == 6); // Downstream saturation freezes I.

    // Legacy gains keep their meaning, even if a scheduled sample runs late.
    PID timing(0, 1, 2, -100, 100, -100, 100);
    CHECK(timing.update(1, 0, 2) == 2);
    CHECK(timing.update(2, 2, 2) == 0); // 2 units over 2 nominal samples.
    CHECK(timing.update(2, 2, 0) == 0);
    CHECK(timing.update(2, 2, std::numeric_limits<float>::quiet_NaN()) == 0);

    PID gyro_damping(0, 0, 60, -1000, 1000, -100, 100);
    // A bias or angle-reference step at rest must not create a D impulse.
    CHECK(gyro_damping.updateWithMeasurementRate(0, -9.5F, 0) == 0);
    CHECK(gyro_damping.updateWithMeasurementRate(4, -7.5F, 0) == 0);
    CHECK(gyro_damping.updateWithMeasurementRate(4, -7.5F, 0.5F) == -30);
    CHECK(gyro_damping.updateWithMeasurementRate(4, -7.5F, -0.5F) == 30);
}
