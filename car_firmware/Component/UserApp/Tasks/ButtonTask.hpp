#pragma once

#include <atomic>
#include "AppTask.hpp"
#include "Button.hpp"
#include "ButtonEventQueue.hpp"
#include "TaskConfig.hpp"

namespace bsp { class BoardHardware; }
namespace app {

class ButtonTask final : public AppTask {
public:
    ButtonTask(bsp::BoardHardware& board, ButtonEventQueue& events)
        : AppTask(task_config::button, stack_, &tcb_), board_(board), events_(events) {}

    [[nodiscard]] uint32_t max_sample_gap_ticks() const
    {
        return max_sample_gap_.load(std::memory_order_relaxed);
    }

private:
    void run() override;
    bsp::BoardHardware& board_;
    ButtonEventQueue& events_;
    Button button_{task_config::button_timing};
    std::atomic<uint32_t> max_sample_gap_{0U};
    StackType_t stack_[task_config::button.stack_depth]{};
    StaticTask_t tcb_{};
};

} // namespace app
