#pragma once

#include <cstdint>

namespace app {

// Records results only; the caller owns the explicit initialization sequence.
// No driver calls, callbacks, allocation, or rollback are hidden in this type.
struct InitializationReport {
    uint32_t attempted_mask{};
    uint32_t failed_mask{};
    bool configuration_valid{true};

    void record(uint8_t id, bool succeeded)
    {
        if (id >= 32U) {
            configuration_valid = false;
            return;
        }

        const uint32_t bit = uint32_t{1} << id;
        if ((attempted_mask & bit) != 0U) {
            // A duplicate result must not erase a previous failure.
            configuration_valid = false;
            failed_mask |= bit;
        }
        attempted_mask |= bit;
        if (!succeeded) {
            failed_mask |= bit;
        }
    }

    [[nodiscard]] bool attempted(uint8_t id) const
    {
        return id < 32U && (attempted_mask & (uint32_t{1} << id)) != 0U;
    }

    [[nodiscard]] bool succeeded(uint8_t id) const
    {
        return attempted(id) && (failed_mask & (uint32_t{1} << id)) == 0U;
    }

    [[nodiscard]] bool all_succeeded(uint32_t required_mask) const
    {
        // Missing an explicit call in main must never enable robot control.
        return configuration_valid && required_mask != 0U && failed_mask == 0U &&
               (attempted_mask & required_mask) == required_mask;
    }
};

} // namespace app
