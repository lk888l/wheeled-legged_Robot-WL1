#include <cmath>
#include <limits>
#include "CtrlAlgorithm/ColdStartLaunch.hpp"
#include "test_check.hpp"

using app::ColdStartLaunch;
using app::ColdStartPhase;
using app::ColdStartAbort;

void advance(ColdStartLaunch& launch, unsigned milliseconds)
{
    for (unsigned i = 0; i < milliseconds; i += 10) { launch.update(10, 0, 0, 0, 0, 0); }
}

int main()
{
    ColdStartLaunch disabled;
    advance(disabled, 12000);
    CHECK(disabled.phase() == ColdStartPhase::complete && !disabled.owns_control());
    CHECK(disabled.height(50) == 50 && disabled.speed(7) == 7);

    ColdStartLaunch launch(true);
    advance(launch, 490);
    CHECK(launch.phase() == ColdStartPhase::waiting);
    launch.update(20, 0, 0, 0, 0, 0); // A delayed sample resets the rest window.
    advance(launch, 490);
    CHECK(launch.phase() == ColdStartPhase::waiting);
    launch.update(10, 0, 0, 0, 30, 0); // Do not arm against a movement request.
    advance(launch, 490);
    CHECK(launch.phase() == ColdStartPhase::waiting);
    advance(launch, 10);
    CHECK(launch.phase() == ColdStartPhase::raising);
    CHECK(launch.height(78.5F) == 44.5F);
    float previous = 44.5F;
    for (unsigned i = 0; i < 300; ++i) {
        advance(launch, 10);
        const float height = launch.height(78.5F);
        CHECK(height >= previous && height - previous < 0.13F);
        CHECK(!launch.balancing() && launch.output_gain() == 0 && launch.speed(90) == 0);
        launch.observe_wheel_rpm(300, 300); // Lifting/handling the wheels is not travel.
        CHECK(launch.distance_mm() == 0);
        previous = height;
    }
    CHECK(launch.phase() == ColdStartPhase::settling && previous == 69.5F);
    advance(launch, 500);
    CHECK(launch.phase() == ColdStartPhase::ramping && launch.output_gain() == 0);
    advance(launch, 500);
    CHECK(launch.output_gain() == 0.5F && launch.speed(0) == -15);
    advance(launch, 500);
    CHECK(launch.phase() == ColdStartPhase::driving && launch.speed(0) == -30);
    CHECK(launch.normal_weight() == 0 && launch.height(44.5F) == 69.5F);

    // 1400 quadrature counts/revolution, 44 mm wheels: about 1216 counts for 12 cm.
    const float one_count_rpm = 1200.0F / 1400.0F;
    launch.observe_wheel_rpm(-200 * one_count_rpm, -200 * one_count_rpm);
    launch.observe_wheel_rpm(200 * one_count_rpm, 200 * one_count_rpm);
    CHECK(std::fabs(launch.distance_mm()) < 0.001F); // Back-and-forth is not progress.
    launch.observe_wheel_rpm(700, -700);
    CHECK(std::fabs(launch.distance_mm()) < 0.001F); // Turning is not forward travel.
    launch.observe_wheel_rpm(-1215 * one_count_rpm, -1215 * one_count_rpm);
    advance(launch, 10);
    CHECK(launch.phase() == ColdStartPhase::driving);
    CHECK(launch.distance_mm() < 120);
    launch.observe_wheel_rpm(-one_count_rpm, -one_count_rpm);
    advance(launch, 10);
    CHECK(launch.phase() == ColdStartPhase::handing_over);
    CHECK(launch.distance_mm() >= 120 && launch.distance_mm() < 120.1F);
    CHECK(launch.height(44.5F) == 69.5F && launch.speed(0) == -30);
    advance(launch, 300);
    CHECK(launch.speed(0) == 0 && launch.height(44.5F) > 44.5F);
    advance(launch, 1200);
    CHECK(launch.phase() == ColdStartPhase::complete && !launch.owns_control());
    CHECK(launch.height(44.5F) == 44.5F && launch.output_gain() == 1);
    advance(launch, 10000);
    CHECK(launch.phase() == ColdStartPhase::complete);

    ColdStartLaunch stuck(true);
    advance(stuck, 9990);
    CHECK(stuck.phase() == ColdStartPhase::driving && stuck.distance_mm() == 0);
    advance(stuck, 10);
    CHECK(stuck.phase() == ColdStartPhase::aborted && stuck.abort_reason() == ColdStartAbort::timeout);
    advance(stuck, 10000);
    CHECK(stuck.phase() == ColdStartPhase::aborted);

    for (const bool delay : {false, true}) {
        ColdStartLaunch unsafe(true);
        advance(unsafe, 500);
        unsafe.update(delay ? 100 : 10, delay ? 0 : 31, 0, 0, 0, 0);
        CHECK(unsafe.phase() == ColdStartPhase::aborted);
        CHECK(unsafe.abort_reason() == (delay ? ColdStartAbort::timing : ColdStartAbort::attitude));
    }
    ColdStartLaunch cancelled(true);
    cancelled.cancel();
    advance(cancelled, 10000);
    CHECK(!cancelled.owns_control() && cancelled.abort_reason() == ColdStartAbort::interrupted);
    ColdStartLaunch invalid(true);
    for (unsigned i = 0; i < 100; ++i) {
        invalid.update(10, std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0);
    }
    CHECK(invalid.phase() == ColdStartPhase::waiting);
}
