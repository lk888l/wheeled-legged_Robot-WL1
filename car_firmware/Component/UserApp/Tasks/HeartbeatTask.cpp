#include "HeartbeatTask.hpp"
#include "BoardHardware.hpp"
#include "RuntimeStatus.hpp"

namespace app {

void HeartbeatTask::run()
{
    app::SystemState previous_state = app::SystemState::booting;
    uint8_t phase = 0U;

    while (true) {
        const app::SystemState state = status_.state();
        if (state != previous_state) {
            previous_state = state;
            phase = 0U;
        }

        bool led_on = false;
        uint32_t duration_ms = 100U;
        switch (state) {
        case app::SystemState::booting:
            led_on = (phase == 0U);
            duration_ms = 100U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        case app::SystemState::ready:
            led_on = (phase == 0U);
            duration_ms = led_on ? 80U : 920U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        case app::SystemState::initialization_failed:
            led_on = (phase == 0U || phase == 2U);
            duration_ms = (phase == 3U) ? 640U : 120U;
            phase = static_cast<uint8_t>((phase + 1U) % 4U);
            break;
        case app::SystemState::task_failed:
            led_on = (phase == 0U || phase == 2U || phase == 4U);
            duration_ms = (phase == 5U) ? 520U : 120U;
            phase = static_cast<uint8_t>((phase + 1U) % 6U);
            break;
        case app::SystemState::runtime_fault:
            led_on = (phase == 0U);
            duration_ms = 80U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        }

        board_.set_status_led(led_on);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

} // namespace app
