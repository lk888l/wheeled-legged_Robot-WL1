#pragma once

#include "AppTask.hpp"
#include "TaskConfig.hpp"

namespace bsp { class BoardHardware; }
namespace app {
class RuntimeStatus;
class ControlState;

class MotionControlTask final : public AppTask {
public:
    MotionControlTask(bsp::BoardHardware& board, RuntimeStatus& status,
                      ControlState& control, AppTask& servo)
        : AppTask(task_config::motion), board_(board), status_(status),
          control_(control), servo_(servo) {}
private:
    void run() override;
    bsp::BoardHardware& board_;
    RuntimeStatus& status_;
    ControlState& control_;
    AppTask& servo_;
};
} // namespace app
