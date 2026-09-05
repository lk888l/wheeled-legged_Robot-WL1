#pragma once

#include <array>
#include <cstdint>
#include "AppTask.hpp"
#include "ButtonEventQueue.hpp"
#include "TaskConfig.hpp"
#include "etl/queue.h"
#include "etl/string.h"
#include "etl/string_view.h"

namespace bsp { class BoardHardware; }
namespace app {
class ButtonTask;
class ControlState;
class RuntimeStatus;

class CommandServiceTask final : public AppTask {
public:
    CommandServiceTask(bsp::BoardHardware& board, RuntimeStatus& status,
                       ControlState& control,
                       ButtonTask& button, ButtonEventQueue& events)
        : AppTask(task_config::command), board_(board), status_(status),
          control_(control), button_(button), events_(events) {}

private:
    void run() override;
    void enqueue_command(etl::string_view frame);
    void process_command(etl::string_view frame);
    void service_periodic();
    void drain_button_events();
    void handle_button_event(ButtonEvent event);

    bsp::BoardHardware& board_;
    RuntimeStatus& status_;
    ControlState& control_;
    ButtonTask& button_;
    ButtonEventQueue& events_;
    etl::queue<etl::string<32U>, 4U> commands_{};
    std::array<uint8_t, 32U> radio_tx_{};
    std::array<uint8_t, 32U> radio_rx_{};
    std::array<uint8_t, 4U> telemetry_sources_{0U, 1U, 2U, 3U};
    std::array<uint32_t, 3U> button_counts_{};
    ButtonEvent last_button_{};
    bool telemetry_enabled_{false};
};

} // namespace app
