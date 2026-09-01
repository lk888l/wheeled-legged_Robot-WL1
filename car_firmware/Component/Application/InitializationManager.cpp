#include "InitializationManager.hpp"

namespace app {

namespace {

constexpr uint8_t kMaximumTrackedSteps = 32U;

} // namespace

bool InitializationReport::attempted(uint8_t id) const
{
    return id < kMaximumTrackedSteps &&
           (attempted_mask & (uint32_t{1} << id)) != 0U;
}

bool InitializationReport::succeeded(uint8_t id) const
{
    if (!attempted(id)) {
        return false;
    }
    return (failed_mask & (uint32_t{1} << id)) == 0U;
}

bool InitializationReport::all_succeeded() const
{
    return configuration_valid && attempted_mask != 0U && failed_mask == 0U;
}

InitializationReport InitializationManager::initialize_all(
    const InitializationStep* steps,
    size_t step_count,
    InitializationObserver observer,
    void* observer_context) const
{
    InitializationReport report{};

    if (steps == nullptr && step_count != 0U) {
        report.configuration_valid = false;
        return report;
    }

    for (size_t index = 0; index < step_count; ++index) {
        const InitializationStep& step = steps[index];
        bool succeeded = false;

        if (step.id >= kMaximumTrackedSteps) {
            report.configuration_valid = false;
        } else {
            const uint32_t bit = uint32_t{1} << step.id;
            if ((report.attempted_mask & bit) != 0U) {
                report.configuration_valid = false;
                report.failed_mask |= bit;
            } else {
                report.attempted_mask |= bit;
                succeeded = step.initialize != nullptr && step.initialize(step.context);
                if (!succeeded) {
                    report.failed_mask |= bit;
                }
            }
        }

        if (observer != nullptr) {
            observer(step, succeeded, observer_context);
        }
    }

    return report;
}

} // namespace app
