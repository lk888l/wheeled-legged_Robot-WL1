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
};

using AppTaskBody = void (*)(void* context);

struct AppTaskConfig {
    TaskId id;
    const char* name;
    configSTACK_DEPTH_TYPE stack_depth;
    UBaseType_t priority;
    AppTaskBody body;
};

/**
 * Small, allocation-free application-task wrapper.
 *
 * The task name and body are fixed at construction, while the context is
 * supplied at start. Task bodies may remain ordinary FreeRTOS-style loops, so
 * legacy control code can be migrated without changing its algorithm.
 */
class AppTask {
public:
    explicit constexpr AppTask(AppTaskConfig config) : config_(config) {}

    AppTask(const AppTask&) = delete;
    AppTask& operator=(const AppTask&) = delete;

    [[nodiscard]] bool start(void* context);
    [[nodiscard]] bool notify_give() const;
    [[nodiscard]] bool notify_value(uint32_t value,
                                    eNotifyAction action = eSetValueWithOverwrite) const;

    [[nodiscard]] TaskHandle_t handle() const { return handle_; }
    [[nodiscard]] TaskId id() const { return config_.id; }
    [[nodiscard]] const char* name() const { return config_.name; }
    [[nodiscard]] bool is_running() const { return handle_ != nullptr; }

private:
    static void task_entry(void* argument);

    AppTaskConfig config_;
    TaskHandle_t handle_{nullptr};
    void* context_{nullptr};
};

} // namespace app
