#include "ButtonTask.hpp"
#include "BoardHardware.hpp"

namespace app {

void ButtonTask::run()
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t previous_sample = last_wake;
    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        const TickType_t gap = now - previous_sample;
        previous_sample = now;
        if (gap > max_sample_gap_.load(std::memory_order_relaxed)) {
            max_sample_gap_.store(gap, std::memory_order_relaxed);
        }
        const Button::Event event = button_.sample(board_.button_pressed(), now);
        if (event != Button::Event::none) {
            (void)events_.push({event, now});
        }

        // When starved, discard missed samples instead of running catch-up
        // iterations with identical GPIO readings. Always block between polls.
        const TickType_t after_sample = xTaskGetTickCount();
        if (TickType_t(after_sample - last_wake) >= task_config::button_period) {
            last_wake = after_sample;
        }
        vTaskDelayUntil(&last_wake, task_config::button_period);
    }
}

} // namespace app
