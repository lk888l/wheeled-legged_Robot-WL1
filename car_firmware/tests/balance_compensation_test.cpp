#include <array>
#include <cmath>

#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "test_check.hpp"

static_assert(BalanceCompensation::clampLegHeight(0.0F) == 44.5F);
static_assert(BalanceCompensation::clampLegHeight(100.0F) == 78.5F);
static_assert(BalanceCompensation::averageLegHeight(44.5F, 78.5F) == 61.5F);
static_assert(BalanceCompensation::pitchBias(9.5F, 44.5F) == 9.5F);

int main()
{
    using namespace BalanceCompensation;
    CHECK(minimum_leg_height_mm == 44.5F);
    CHECK(maximum_leg_height_mm == 78.5F);
    CHECK(default_minimum_bias_degrees == 9.5F);

    // Reference points were calculated independently in double precision from
    // 9.5 + f(h) - f(44.5); they do not call the production helper for expected values.
    struct Reference { float height; double bias; };
    constexpr std::array<Reference, 5U> reference{{
        {44.5F, 9.5}, {50.0F, 7.913635}, {60.0F, 6.619635},
        {61.5F, 6.602520}, {78.5F, 9.635320}}};
    for (const auto& point : reference) {
        CHECK(std::abs(pitchBias(9.5F, point.height) - point.bias) < 0.00002);
        CHECK(std::abs(pitchBias(12.0F, point.height) - (point.bias + 2.5)) < 0.00002);
        CHECK(clampLegHeight(point.height) == point.height);
    }
    for (const float baseline : {-12.5F, 0.0F, 9.5F, 23.5F}) {
        CHECK(pitchBias(baseline, minimum_leg_height_mm) == baseline);
    }

    const float low = clampLegHeight(-100.0F);
    const float high = clampLegHeight(100.0F);
    CHECK(low == 44.5F && high == 78.5F);
    CHECK(averageLegHeight(low, high) == 61.5F);
    CHECK(averageLegHeight(low, high) == averageLegHeight(high, low));
    CHECK(pitchBias(9.5F, averageLegHeight(low, high)) ==
          pitchBias(9.5F, averageLegHeight(high, low)));
}
