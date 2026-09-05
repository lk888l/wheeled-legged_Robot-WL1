#pragma once

#include "AppTask.hpp"
#include "TaskConfig.hpp"

namespace bsp { class BoardHardware; }
namespace app {
class RuntimeStatus;

class HeartbeatTask final : public AppTask {
public:
    HeartbeatTask(bsp::BoardHardware& board, RuntimeStatus& status)
        : AppTask(task_config::heartbeat), board_(board), status_(status) {}
private:
    void run() override;
    bsp::BoardHardware& board_;
    RuntimeStatus& status_;
};
} // namespace app
