#include "Button.hpp"

namespace app {

Button::Event Button::sample(bool pressed, uint32_t now) noexcept
{
    Event event = Event::none;
    if (pressed != raw_pressed_) {
        raw_pressed_ = pressed;
        raw_changed_at_ = now;
    }

    if (raw_pressed_ != stable_pressed_ &&
        uint32_t(now - raw_changed_at_) >= timing_.debounce) {
        stable_pressed_ = raw_pressed_;
        if (stable_pressed_) {
            pressed_at_ = now;
            long_emitted_ = false;
            second_press_ = click_pending_ &&
                uint32_t(now - released_at_) <= timing_.double_click;
            if (click_pending_ && !second_press_) {
                event = Event::click;
            }
            click_pending_ = false;
        } else {
            if (!long_emitted_) {
                // Release debounce must not extend a hold. A late sample can
                // first observe release after the long-press threshold.
                if (uint32_t(raw_changed_at_ - pressed_at_) >= timing_.long_press) {
                    event = Event::long_press;
                } else if (second_press_) {
                    event = Event::double_click;
                } else {
                    click_pending_ = true;
                    released_at_ = now;
                }
            }
            second_press_ = false;
        }
    }

    if (stable_pressed_ && raw_pressed_ && !long_emitted_ &&
        uint32_t(now - pressed_at_) >= timing_.long_press) {
        long_emitted_ = true;
        second_press_ = false;
        event = Event::long_press;
    }

    if (click_pending_ && !stable_pressed_ &&
        uint32_t(now - released_at_) >= timing_.double_click) {
        click_pending_ = false;
        event = Event::click;
    }
    return event;
}

} // namespace app
