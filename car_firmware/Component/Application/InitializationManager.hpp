#pragma once

#include <cstddef>
#include <cstdint>

namespace app {

using InitializationFunction = bool (*)(void* context);

struct InitializationStep {
    uint8_t id;
    const char* name;
    InitializationFunction initialize;
    void* context;
};

struct InitializationReport {
    uint32_t attempted_mask{};
    uint32_t failed_mask{};
    bool configuration_valid{true};

    [[nodiscard]] bool attempted(uint8_t id) const;
    [[nodiscard]] bool succeeded(uint8_t id) const;
    [[nodiscard]] bool all_succeeded() const;
};

using InitializationObserver = void (*)(const InitializationStep& step,
                                         bool succeeded,
                                         void* context);

/**
 * Runs every initialization step in declaration order.
 *
 * A failed step is reported immediately and recorded, but never short-circuits
 * the remaining steps. This is intentionally different from a transactional
 * startup/rollback manager: the robot must retain diagnostics and independent
 * command paths even when a hardware device is absent.
 */
class InitializationManager {
public:
    [[nodiscard]] InitializationReport initialize_all(
        const InitializationStep* steps,
        size_t step_count,
        InitializationObserver observer = nullptr,
        void* observer_context = nullptr) const;
};

} // namespace app
