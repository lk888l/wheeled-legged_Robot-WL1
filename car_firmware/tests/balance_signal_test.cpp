#include "CtrlAlgorithm/BalanceSignal.hpp"
#include "test_check.hpp"
#include <algorithm>

int main()
{
    constexpr double radians = 0.017453292519943295;
    double gyro[3] = {2, 10 * radians, 0};
    CHECK(std::abs(BalanceSignal::pitchRate(0, gyro) - 10) < 0.0001F);
    gyro[1] = 0;
    gyro[2] = 10 * radians;
    CHECK(std::abs(BalanceSignal::pitchRate(30, gyro) + 5) < 0.0001F);
    CHECK(std::abs(BalanceSignal::pitchRate(-30, gyro) - 5) < 0.0001F);

    BalanceSignal::WheelFilter filter;
    // One 50 ms encoder count is 60,000 / (1400 * 50) = 6/7 RPM.
    // Alternating quantization at zero must not retain its full amplitude.
    float peak = 0;
    for (int i = 0; i < 100; ++i) {
        const float value = filter.update((i % 2 ? -1 : 1) * 6.0F / 7, 50);
        peak = std::max(peak, std::abs(value));
    }
    CHECK(peak < 0.6F);
    for (int i = 0; i < 20; ++i) filter.update(20, 50);
    CHECK(std::abs(filter.update(20, 50) - 20) < 0.001F);
    filter.reset();
    CHECK(filter.update(0, 50) == 0);
    CHECK(filter.update(10, 0) == 0);
    CHECK(std::abs(filter.update(10, 25) - 5) < 0.0001F);
}
