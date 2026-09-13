#include <cmath>
#include <limits>
#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "CtrlAlgorithm/BalanceStartupGate.hpp"
#include "test_check.hpp"

int main()
{
    BalanceStartupGate gate;
    const auto arm = [&] {
        for (unsigned i = 0; i < 49; ++i) { CHECK(!gate.update(true, 0, 0, 0, 0, 0)); }
        CHECK(gate.update(true, 0, 0, 0, 0, 0));
    };
    for (unsigned i = 0; i < 100; ++i) {
        CHECK(!gate.update(true, 9, 0, 0, 0, 0));
        CHECK(!gate.update(true, 0, 6, 0, 0, 0));
        CHECK(!gate.update(true, 0, 0, 21, 0, 0));
        CHECK(!gate.update(true, 0, 0, 0, 1, 0));
        CHECK(!gate.update(true, 0, 0, 0, 0, 1));
    }
    arm();
    CHECK(gate.update(true, 10, 18, 30, 50, 20));
    CHECK(!gate.update(true, 31, 0, 0, 0, 0));
    arm();
    CHECK(!gate.update(true, 0, -31, 0, 0, 0));
    arm();
    CHECK(!gate.update(false, 0, 0, 0, 0, 0));
    arm();
    CHECK(!gate.update(true, std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0));
    arm();
    CHECK(!gate.update(true, 0, 0, 0, std::numeric_limits<float>::infinity(), 0));
    using namespace BalanceCompensation;
    CHECK(clampLegHeight(0) == 44.5F && clampLegHeight(100) == 78.5F);
    CHECK(pitchBias(9.5F, 44.5F) == 9.5F);
    CHECK(std::fabs(pitchBias(9.5F, 61.5F) - 6.60252F) < 0.0001F);
    CHECK(std::fabs(pitchBias(10.5F, 61.5F) - pitchBias(9.5F, 61.5F) - 1) < 0.0001F);
    CHECK(averageLegHeight(50, 73) == averageLegHeight(73, 50));
}
