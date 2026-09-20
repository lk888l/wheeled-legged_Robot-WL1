#pragma once

#include "AppTask.hpp"
#include "TaskConfig.hpp"

namespace bsp { class BoardHardware; }
namespace app {
class RuntimeStatus;
class ControlState;

class MotionControlTask final : public AppTask {
public:
    // Production uses ordinary startup. A coldstart command explicitly starts
    // the stand launch; test compositions may opt in at construction.
    MotionControlTask(bsp::BoardHardware& board, RuntimeStatus& status,
                      ControlState& control, AppTask& servo, bool cold_start_enabled = false)
        : AppTask(task_config::motion), board_(board), status_(status),
          control_(control), servo_(servo), cold_start_enabled_(cold_start_enabled) {}
private:
    void run() override;
    bsp::BoardHardware& board_;
    RuntimeStatus& status_;
    ControlState& control_;
    AppTask& servo_;
    const bool cold_start_enabled_;
};
} // namespace app
