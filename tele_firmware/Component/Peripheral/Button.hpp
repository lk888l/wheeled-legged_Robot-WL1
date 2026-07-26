#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

class Button final {
public:
    enum class Event : std::uint8_t {
        none,
        clicked,
        long_pressed,
    };

    static constexpr std::uint8_t debounce_samples = 4U;
    static constexpr TickType_t long_press_ticks = pdMS_TO_TICKS(1000U);

    [[nodiscard]] Event sample(bool pressed, TickType_t now) noexcept;

private:
    std::uint8_t debounce_count_ = 0U;
    bool stable_pressed_ = false;
    bool long_press_emitted_ = false;
    TickType_t pressed_at_ = 0U;
};
