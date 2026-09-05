#include "AppTask.hpp"

namespace app {

bool AppTask::start()
{
    if (handle_ != nullptr || config_.name == nullptr || config_.name[0] == '\0' ||
        config_.stack_depth == 0U || config_.priority >= configMAX_PRIORITIES ||
        ((stack_ == nullptr) != (tcb_ == nullptr))) {
        return false;
    }

    // Publish the static task handle before run() can observe it. Interrupts
    // remain enabled; scheduler suspension occurs only during startup.
    vTaskSuspendAll();
    if (stack_ != nullptr) {
        handle_ = xTaskCreateStatic(task_entry, config_.name, config_.stack_depth,
                                   this, config_.priority, stack_, tcb_);
    } else if (xTaskCreate(task_entry, config_.name, config_.stack_depth,
                           this, config_.priority, &handle_) != pdPASS) {
        handle_ = nullptr;
    }
    const bool started = handle_ != nullptr;
    (void)xTaskResumeAll();
    return started;
}

bool AppTask::notify_give() const
{
    return handle_ != nullptr && xTaskNotifyGive(handle_) == pdPASS;
}

bool AppTask::notify_value(uint32_t value, eNotifyAction action) const
{
    if (handle_ == nullptr) {
        return false;
    }
    return xTaskNotify(handle_, value, action) == pdPASS;
}

void AppTask::task_entry(void* argument)
{
    auto& task = *static_cast<AppTask*>(argument);
    task.run();
    configASSERT(false); // Persistent tasks must not invalidate their handles.
    for (;;) {
        vTaskSuspend(nullptr);
    }
}

} // namespace app
