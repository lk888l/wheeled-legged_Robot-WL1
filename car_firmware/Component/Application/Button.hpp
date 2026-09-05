#pragma once

#include <cstdint>

namespace app {

/** Allocation-free, O(1) recognizer, independent of GPIO and RTOS.
 * Pass logical pressed=true and a wrapping uint32_t clock. Durations use that
 * clock's units and must be positive, below half its range. Sample at least
 * once per debounce interval. Long press overrides clicks, including a held
 * second press. A boot-held key is treated as a normal debounced press.
 * Each sample emits at most one event.
 */
class Button final {
public:
    enum class Event : uint8_t { none, click, double_click, long_press };
    struct Timing {
        uint32_t debounce;
        uint32_t double_click;
        uint32_t long_press;
    };

    explicit constexpr Button(Timing timing) : timing_(timing) {}
    [[nodiscard]] Event sample(bool pressed, uint32_t now) noexcept;

private:
    const Timing timing_;
    uint32_t raw_changed_at_{0U};
    uint32_t pressed_at_{0U};
    uint32_t released_at_{0U};
    bool raw_pressed_{false};
    bool stable_pressed_{false};
    bool click_pending_{false};
    bool second_press_{false};
    bool long_emitted_{false};
};

} // namespace app
