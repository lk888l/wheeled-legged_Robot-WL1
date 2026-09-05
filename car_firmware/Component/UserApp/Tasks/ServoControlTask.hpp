#pragma once

#include "AppTask.hpp"
#include "TaskConfig.hpp"

namespace bsp { class BoardHardware; }
namespace app {
class RuntimeStatus;
class ControlState;

class ServoControlTask final : public AppTask {
public:
    ServoControlTask(bsp::BoardHardware& board, RuntimeStatus& status, ControlState& control)
        : AppTask(task_config::servo), board_(board), status_(status), control_(control) {}
private:
    void run() override;
    bsp::BoardHardware& board_;
    RuntimeStatus& status_;
    ControlState& control_;
};
} // namespace app
