/********************************************************************************
  * @file           : Button.hpp
  * @brief          : Debounced FreeRTOS button event publisher.
  *******************************************************************************/

#ifndef __TELE_BUTTON_HPP
#define __TELE_BUTTON_HPP

#include <cstdint>
#include <functional>
#include <type_traits>

#include "BasicObject.hpp"

class Button : public BasicObject {
public:
    static constexpr std::uint8_t DEBOUNCE_SAMPLES = 4U;
    static constexpr TickType_t LONG_PRESS_TICKS = pdMS_TO_TICKS(1000U);

    void sample(bool pressed, TickType_t now);

    template<typename SignalPtr>
    bool bindReactor(SignalPtr signalFunc, TaskHandle_t task, std::uint32_t bitMask) {
        if constexpr (std::is_same_v<SignalPtr, decltype(&Button::signal_click)>) {
            if (signalFunc == &Button::signal_click) {
                click_cfg_.task_h = task;
                click_cfg_.bitMask = bitMask;
                return true;
            }
        } else if constexpr (std::is_same_v<SignalPtr, decltype(&Button::signal_long_press)>) {
            if (signalFunc == &Button::signal_long_press) {
                long_press_cfg_.task_h = task;
                long_press_cfg_.bitMask = bitMask;
                return true;
            }
        }
        return false;
    }

    void signal_click(const std::function<void()>& slot) const;
    void signal_long_press(const std::function<void()>& slot) const;

private:
    std::uint8_t debounce_count_ = 0U;
    bool stable_pressed_ = false;
    bool long_press_emitted_ = false;
    TickType_t pressed_at_ = 0U;

    SignalContext click_cfg_{};
    SignalContext long_press_cfg_{};
};

#endif // __TELE_BUTTON_HPP
