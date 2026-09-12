#include <array>
#include <cmath>

#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "test_check.hpp"

static_assert(BalanceCompensation::clampLegHeight(0.0F) == 44.5F);
static_assert(BalanceCompensation::clampLegHeight(100.0F) == 78.5F);
static_assert(BalanceCompensation::averageLegHeight(44.5F, 78.5F) == 61.5F);
static_assert(BalanceCompensation::pitchBias(
    BalanceCompensation::default_minimum_bias_degrees, 44.5F) ==
    BalanceCompensation::default_minimum_bias_degrees);

int main()
{
    using namespace BalanceCompensation;
    CHECK(minimum_leg_height_mm == 44.5F);
    CHECK(maximum_leg_height_mm == 78.5F);
    CHECK(default_minimum_bias_degrees == 7.0F);

    // Reference points were calculated independently in double precision from
    // 7.0 + f(h) - f(44.5); they do not call the production helper for expected values.
    struct Reference { float height; double bias; };
    constexpr std::array<Reference, 5U> reference{{
        {44.5F, 7.0}, {50.0F, 5.413635}, {60.0F, 4.119635},
        {61.5F, 4.102520}, {78.5F, 7.135320}}};
    for (const auto& point : reference) {
        CHECK(std::abs(pitchBias(7.0F, point.height) - point.bias) < 0.00002);
        CHECK(std::abs(pitchBias(12.0F, point.height) - (point.bias + 5.0)) < 0.00002);
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
    CHECK(pitchBias(default_minimum_bias_degrees, averageLegHeight(low, high)) ==
          pitchBias(default_minimum_bias_degrees, averageLegHeight(high, low)));
}
