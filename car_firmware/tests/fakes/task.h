#pragma once
#include <string>
#include <vector>
#include "FreeRTOS.h"

enum eNotifyAction { eNoAction, eSetBits, eIncrement, eSetValueWithOverwrite, eSetValueWithoutOverwrite };

namespace fake_rtos {
struct LoopDone {};
struct TaskCreation {
    std::string name;
    configSTACK_DEPTH_TYPE stack_depth;
    UBaseType_t priority;
    bool uses_static_storage;
    TickType_t tick;
};
inline std::vector<TaskCreation> creations;
inline std::vector<TickType_t> delays;
inline bool (*on_create)(const char*){};
inline void (*on_resume)(){};
inline int token{};
inline unsigned dynamic_creates{}, static_creates{}, notifications{};
inline unsigned scheduler_depth{};
inline bool fail_create{}, run_on_resume{};
inline TaskFunction_t entry{};
inline void* argument{};
inline StackType_t* supplied_stack{};
inline StaticTask_t* supplied_tcb{};
inline TaskHandle_t notified_handle{};
inline uint32_t notified_value{}, pending_notifications{};
inline eNotifyAction notified_action{};
inline TickType_t now{};
inline void (*on_delay)(){};
inline BaseType_t (*on_wait)(){};
inline unsigned nonblocking_delays{};

inline void reset()
{
    creations.clear();
    delays.clear();
    on_create = nullptr;
    on_resume = nullptr;
    critical_depth = 0;
    critical_entries = 0U;
    dynamic_creates = static_creates = notifications = scheduler_depth = 0U;
    fail_create = run_on_resume = false;
    entry = nullptr;
    argument = nullptr;
    supplied_stack = nullptr;
    supplied_tcb = nullptr;
    notified_handle = nullptr;
    notified_value = pending_notifications = now = nonblocking_delays = 0U;
    on_delay = nullptr;
    on_wait = nullptr;
}
}

inline void vTaskSuspendAll() { ++fake_rtos::scheduler_depth; }
inline BaseType_t xTaskResumeAll()
{
    assert(fake_rtos::scheduler_depth > 0U);
    --fake_rtos::scheduler_depth;
    if (fake_rtos::run_on_resume) {
        fake_rtos::run_on_resume = false;
        fake_rtos::entry(fake_rtos::argument);
    }
    if (fake_rtos::on_resume != nullptr) { fake_rtos::on_resume(); }
    return pdTRUE;
}
inline BaseType_t xTaskCreate(TaskFunction_t entry, const char* name, configSTACK_DEPTH_TYPE stack_depth,
                               void* argument, UBaseType_t priority, TaskHandle_t* handle)
{
    assert(fake_rtos::scheduler_depth > 0U);
    ++fake_rtos::dynamic_creates;
    fake_rtos::creations.push_back({name, stack_depth, priority, false, fake_rtos::now});
    if (fake_rtos::fail_create ||
        (fake_rtos::on_create != nullptr && !fake_rtos::on_create(name))) { return pdFALSE; }
    fake_rtos::entry = entry;
    fake_rtos::argument = argument;
    *handle = &fake_rtos::token;
    return pdPASS;
}
inline TaskHandle_t xTaskCreateStatic(TaskFunction_t entry, const char* name, configSTACK_DEPTH_TYPE stack_depth,
                                      void* argument, UBaseType_t priority, StackType_t* stack, StaticTask_t* tcb)
{
    assert(fake_rtos::scheduler_depth > 0U);
    ++fake_rtos::static_creates;
    fake_rtos::creations.push_back({name, stack_depth, priority, true, fake_rtos::now});
    if (fake_rtos::fail_create ||
        (fake_rtos::on_create != nullptr && !fake_rtos::on_create(name))) { return nullptr; }
    fake_rtos::entry = entry;
    fake_rtos::argument = argument;
    fake_rtos::supplied_stack = stack;
    fake_rtos::supplied_tcb = tcb;
    return &fake_rtos::token;
}
inline BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, eNotifyAction action)
{
    ++fake_rtos::notifications;
    fake_rtos::notified_handle = handle;
    fake_rtos::notified_value = value;
    fake_rtos::notified_action = action;
    return pdPASS;
}
inline BaseType_t xTaskNotifyGive(TaskHandle_t handle) { return xTaskNotify(handle, 0U, eIncrement); }
inline BaseType_t xTaskNotifyFromISR(TaskHandle_t handle, uint32_t value, eNotifyAction action, BaseType_t*)
{ return xTaskNotify(handle, value, action); }
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return &fake_rtos::token; }
inline TickType_t xTaskGetTickCount() { return fake_rtos::now; }
inline BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t* value, TickType_t)
{
    if (fake_rtos::pending_notifications == 0U) {
        if (fake_rtos::on_wait != nullptr) { return fake_rtos::on_wait(); }
        throw fake_rtos::LoopDone{};
    }
    *value = fake_rtos::pending_notifications;
    fake_rtos::pending_notifications = 0U;
    return pdTRUE;
}
inline void vTaskSuspend(TaskHandle_t) { throw fake_rtos::LoopDone{}; }
inline void vTaskDelay(TickType_t ticks)
{
    fake_rtos::delays.push_back(ticks);
    fake_rtos::now += ticks;
    if (fake_rtos::on_delay != nullptr) { fake_rtos::on_delay(); }
}
inline uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t)
{
    if (fake_rtos::pending_notifications == 0U) { throw fake_rtos::LoopDone{}; }
    const auto pending = fake_rtos::pending_notifications;
    if (clear_on_exit == pdTRUE) { fake_rtos::pending_notifications = 0U; }
    else { --fake_rtos::pending_notifications; }
    return pending;
}
inline void vTaskDelayUntil(TickType_t* previous, TickType_t period)
{
    *previous += period;
    if (*previous > fake_rtos::now) { fake_rtos::now = *previous; }
    else { ++fake_rtos::nonblocking_delays; }
    if (fake_rtos::on_delay != nullptr) { fake_rtos::on_delay(); }
}
