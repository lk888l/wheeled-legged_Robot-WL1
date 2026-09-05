#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

namespace app {

enum class TaskId : uint8_t {
    heartbeat = 0,
    command_service,
    servo_control,
    motion_control,
    button,
};

struct AppTaskConfig {
    TaskId id;
    const char* name;
    configSTACK_DEPTH_TYPE stack_depth; // StackType_t words, not bytes.
    UBaseType_t priority;
};

/**
 * Persistent application-task interface. Dependencies belong to subclasses.
 * Construct at static lifetime, then start once from the bootstrap task.
 * Never externally delete or destroy a running task. run() must block
 * periodically and must not return. Notification methods are task-only.
 * Optional caller-owned stack/TCB avoids startup allocation; otherwise the
 * existing tasks retain startup-only dynamic allocation and memory budgets.
 */
class AppTask {
public:
    AppTask(const AppTask&) = delete;
    AppTask& operator=(const AppTask&) = delete;

    AppTask(AppTask&&) = delete;
    AppTask& operator=(AppTask&&) = delete;

    [[nodiscard]] bool start();
    [[nodiscard]] bool notify_give() const;
    [[nodiscard]] bool notify_value(uint32_t value,
                                    eNotifyAction action = eSetValueWithOverwrite) const;

    [[nodiscard]] TaskHandle_t handle() const { return handle_; }
    [[nodiscard]] TaskId id() const { return config_.id; }
    [[nodiscard]] const char* name() const { return config_.name; }
    [[nodiscard]] bool is_running() const { return handle_ != nullptr; }

protected:
    explicit AppTask(AppTaskConfig config, StackType_t* stack = nullptr,
                     StaticTask_t* tcb = nullptr)
        : config_(config), stack_(stack), tcb_(tcb) {}
    // No polymorphic deletion: objects and storage outlive the scheduler.
    ~AppTask() = default;
    virtual void run() = 0;

private:
    static void task_entry(void* argument);

    const AppTaskConfig config_;
    StackType_t* const stack_;
    StaticTask_t* const tcb_;
    TaskHandle_t handle_{nullptr};
};

} // namespace app
