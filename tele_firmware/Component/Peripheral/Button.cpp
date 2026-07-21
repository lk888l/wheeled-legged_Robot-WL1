/********************************************************************************
  * @file           : Button.cpp
  * @brief          : Debounced FreeRTOS button event publisher.
  *******************************************************************************/

#include "Button.hpp"

void Button::sample(bool pressed, TickType_t now)
{
    if (pressed != stable_pressed_) {
        if (++debounce_count_ < DEBOUNCE_SAMPLES) {
            return;
        }

        debounce_count_ = 0U;
        stable_pressed_ = pressed;
        if (stable_pressed_) {
            long_press_emitted_ = false;
            pressed_at_ = now;
        } else if (!long_press_emitted_) {
            emit(click_cfg_);
        }
        return;
    }

    debounce_count_ = 0U;
    if (stable_pressed_ && !long_press_emitted_ &&
        static_cast<TickType_t>(now - pressed_at_) >= LONG_PRESS_TICKS) {
        long_press_emitted_ = true;
        emit(long_press_cfg_);
    }
}

void Button::signal_click(const std::function<void()>& slot) const
{
    if (slot) {
        slot();
    }
}

void Button::signal_long_press(const std::function<void()>& slot) const
{
    if (slot) {
        slot();
    }
}
