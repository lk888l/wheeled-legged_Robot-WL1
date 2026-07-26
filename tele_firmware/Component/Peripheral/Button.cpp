#include "Button.hpp"

Button::Event Button::sample(bool pressed, TickType_t now) noexcept
{
    if (pressed != stable_pressed_) {
        if (++debounce_count_ < debounce_samples) {
            return Event::none;
        }

        debounce_count_ = 0U;
        stable_pressed_ = pressed;
        if (stable_pressed_) {
            long_press_emitted_ = false;
            pressed_at_ = now;
            return Event::none;
        }

        return long_press_emitted_ ? Event::none : Event::clicked;
    }

    debounce_count_ = 0U;
    if (stable_pressed_ && !long_press_emitted_ &&
        static_cast<TickType_t>(now - pressed_at_) >= long_press_ticks) {
        long_press_emitted_ = true;
        return Event::long_pressed;
    }

    return Event::none;
}
