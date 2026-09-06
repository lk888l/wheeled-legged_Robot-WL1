#include "vqf.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static constexpr double radians = 0.017453292519943295;

static void checkStationary(double roll_degrees, double pitch_degrees)
{
    VQF fusion(0.01);
    const double roll = roll_degrees * radians;
    const double pitch = pitch_degrees * radians;
    const vqf_real_t gyro[3] = {0.0, 0.0, 0.0};
    const vqf_real_t gravity[3] = {
        -9.81 * std::sin(pitch),
        9.81 * std::sin(roll) * std::cos(pitch),
        9.81 * std::cos(roll) * std::cos(pitch),
    };

    // Include the first sample and the transition out of the filter warm-up.
    for (unsigned sample = 0; sample < 1000; ++sample) {
        fusion.update(gyro, gravity);
        vqf_real_t q[4];
        fusion.getQuat6D(q);
        const double actual_roll = std::atan2(2.0 * (q[0]*q[1] + q[2]*q[3]),
            q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) / radians;
        const double actual_pitch = -std::asin(2.0 * (q[1]*q[3] - q[0]*q[2])) / radians;
        if (!std::isfinite(actual_roll) || !std::isfinite(actual_pitch) ||
            std::fabs(actual_roll - roll_degrees) > 0.02 ||
            std::fabs(actual_pitch - pitch_degrees) > 0.02) {
            std::fprintf(stderr,
                "FAIL: sample=%u expected roll/pitch=%.3f/%.3f actual=%.3f/%.3f\n",
                sample, roll_degrees, pitch_degrees, actual_roll, actual_pitch);
            std::exit(EXIT_FAILURE);
        }
    }
}

int main()
{
    checkStationary(0.0, 0.0);
    checkStationary(0.0, -12.6);
    checkStationary(10.0, -12.6);
    checkStationary(-10.0, -45.0);
    std::puts("PASS: VQF startup and 10 s stationary attitude at four orientations");
}
