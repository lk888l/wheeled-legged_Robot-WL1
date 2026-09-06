#include "CtrlAlgorithm/BalanceCompensation.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace BC = BalanceCompensation;

static void require(bool condition, const char* scenario)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", scenario);
        std::exit(EXIT_FAILURE);
    }
}

static void expectNear(float actual, float expected, const char* scenario)
{
    require(BC::isFiniteBias(actual) && std::fabs(actual - expected) < 0.0001F, scenario);
}

int main()
{
    require(BC::minimum_leg_height_mm == 44.5F, "minimum height remains 44.5 mm");
    require(BC::maximum_leg_height_mm == 78.5F, "maximum height remains 78.5 mm");
    require(BC::default_minimum_bias_degrees == 12.6F, "reset calibration is 12.6 degrees");

    // Independent reference values from the accepted calibration table.
    struct Reference { float height; float bias_12_6; float bias_13_6; };
    const std::array references{
        Reference{44.5F, 12.6F, 13.6F},
        Reference{61.5F, 9.70252F, 10.70252F},
        Reference{78.5F, 12.73532F, 13.73532F},
    };
    for (const auto& reference : references) {
        expectNear(BC::pitchBias(12.6F, reference.height), reference.bias_12_6,
                   "default baseline follows the reference curve");
        expectNear(BC::pitchBias(13.6F, reference.height), reference.bias_13_6,
                   "edited baseline follows the reference curve");
    }

    // Height compensation must never change the meaning of the minimum baseline.
    for (float baseline : {-5.0F, 0.0F, 12.6F, 13.6F, 20.0F}) {
        require(BC::pitchBias(baseline, 44.5F) == baseline,
                "minimum height uses the exact calibrated baseline");
        for (unsigned step = 0; step <= 340; ++step) {
            const float height = 44.5F + 0.1F * static_cast<float>(step);
            expectNear(BC::pitchBias(baseline + 1.0F, height) - BC::pitchBias(baseline, height),
                       1.0F, "editing the baseline shifts the whole curve by the same amount");
        }
    }

    expectNear(BC::averageLegHeight(44.5F, 78.5F), 61.5F, "both legs contribute to the average");
    expectNear(BC::averageLegHeight(78.5F, 44.5F), 61.5F, "left/right exchange keeps the average");
    expectNear(BC::pitchBias(13.6F, BC::averageLegHeight(44.5F, 78.5F)), 10.70252F,
               "asymmetric legs use their shared average for compensation");

    // Limit each leg before averaging: clamp(avg(20,80)) would incorrectly give 50.
    const float limited_left = BC::clampLegHeight(20.0F);
    const float limited_right = BC::clampLegHeight(80.0F);
    expectNear(limited_left, 44.5F, "low leg target is clamped");
    expectNear(limited_right, 78.5F, "high leg target is clamped");
    expectNear(BC::averageLegHeight(limited_left, limited_right), 61.5F,
               "per-leg bounds are applied before averaging");
    expectNear(BC::clampLegHeight(55.0F), 55.0F, "valid leg target is preserved");
    expectNear(BC::pitchBias(12.6F, BC::averageLegHeight(
                   BC::clampLegHeight(0.0F), BC::clampLegHeight(0.0F))),
               12.6F, "below-minimum targets use the minimum-height calibration");

    // Repeated evaluations after a change at middle height must not accumulate trim.
    for (unsigned cycle = 0; cycle < 1000; ++cycle) {
        const auto& reference = references[cycle % references.size()];
        expectNear(BC::pitchBias(13.6F, reference.height), reference.bias_13_6,
                   "calibration is stable over repeated height changes");
    }
    expectNear(BC::pitchBias(13.6F, 44.5F), 13.6F, "lowering the legs returns to the edited baseline");

    require(BC::isFiniteBias(13.6F) && BC::isFiniteBias(-5.0F), "finite calibration values are accepted");
    require(!BC::isFiniteBias(std::numeric_limits<float>::quiet_NaN()), "NaN calibration is rejected");
    require(!BC::isFiniteBias(std::numeric_limits<float>::infinity()), "positive infinity is rejected");
    require(!BC::isFiniteBias(-std::numeric_limits<float>::infinity()), "negative infinity is rejected");

    std::puts("PASS: balance calibration, height limits, symmetry and repeated updates");
}
