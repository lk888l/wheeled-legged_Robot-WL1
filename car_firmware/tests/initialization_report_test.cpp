#include <initializer_list>

#include "InitializationReport.hpp"
#include "test_check.hpp"

int main()
{
    constexpr uint32_t required = 0x07U;
    app::InitializationReport report;
    CHECK(!report.all_succeeded(required));
    CHECK(!report.all_succeeded(0U));
    CHECK(!report.attempted(0U) && !report.succeeded(0U));
    report.record(0U, true);
    report.record(2U, true);
    CHECK(!report.all_succeeded(required)); // Missing IMU cannot pass startup.
    report.record(1U, true);
    CHECK(report.all_succeeded(required));
    CHECK(report.attempted_mask == required && report.failed_mask == 0U);

    // Each failed module is retained; later independent results still count.
    for (uint8_t failed = 0U; failed < 3U; ++failed) {
        app::InitializationReport partial;
        for (uint8_t id = 0U; id < 3U; ++id) {
            partial.record(id, id != failed);
        }
        CHECK(partial.configuration_valid);
        CHECK(partial.attempted_mask == required);
        CHECK(partial.failed_mask == (uint32_t{1} << failed));
        CHECK(!partial.succeeded(failed));
        CHECK(!partial.all_succeeded(required));
    }

    // A duplicate, even a successful retry, invalidates the startup sequence.
    for (const bool first_succeeded : {false, true}) {
        app::InitializationReport duplicate;
        duplicate.record(0U, first_succeeded);
        duplicate.record(0U, true);
        duplicate.record(1U, true);
        CHECK(!duplicate.configuration_valid);
        CHECK(duplicate.attempted_mask == 3U && duplicate.failed_mask == 1U);
        CHECK(duplicate.succeeded(1U) && !duplicate.succeeded(0U));
        CHECK(!duplicate.all_succeeded(3U));
    }

    app::InitializationReport boundary;
    boundary.record(31U, true);
    CHECK(boundary.succeeded(31U));
    CHECK(boundary.all_succeeded(uint32_t{1} << 31U));
    for (const uint8_t invalid : {uint8_t{32U}, uint8_t{255U}}) {
        boundary.record(invalid, true);
        CHECK(!boundary.configuration_valid);
        CHECK(!boundary.attempted(invalid) && !boundary.succeeded(invalid));
        CHECK(!boundary.all_succeeded(uint32_t{1} << 31U));
    }
    boundary.record(0U, true);
    CHECK(boundary.succeeded(0U)); // Invalid IDs do not block later recording.
    CHECK(boundary.attempted_mask == 0x80000001U);

    app::InitializationReport extra_failure;
    extra_failure.record(0U, true);
    extra_failure.record(1U, false);
    CHECK(!extra_failure.all_succeeded(1U)); // Do not hide other failures.
}
