#include "AppTask.hpp"

namespace app {

bool AppTask::start(void* context)
{
    if (handle_ != nullptr || config_.name == nullptr || config_.body == nullptr) {
        return false;
    }

    context_ = context;
    const BaseType_t result = xTaskCreate(task_entry,
                                          config_.name,
                                          config_.stack_depth,
                                          this,
                                          config_.priority,
                                          &handle_);
    if (result != pdPASS) {
        handle_ = nullptr;
        context_ = nullptr;
        return false;
    }
    return true;
}

bool AppTask::notify_give() const
{
    if (handle_ == nullptr) {
        return false;
    }
    xTaskNotifyGive(handle_);
    return true;
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
    auto* task = static_cast<AppTask*>(argument);
    if (task != nullptr && task->config_.body != nullptr) {
        task->config_.body(task->context_);
    }

    if (task != nullptr) {
        task->handle_ = nullptr;
    }
    vTaskDelete(nullptr);
}

} // namespace app
