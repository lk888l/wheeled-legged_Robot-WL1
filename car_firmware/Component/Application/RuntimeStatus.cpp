#include "RuntimeStatus.hpp"
#include "CriticalSection.hpp"

extern "C" {
volatile uint32_t g_app_system_state =
    static_cast<uint32_t>(app::SystemState::booting);
volatile uint32_t g_app_hardware_attempted_mask = 0U;
volatile uint32_t g_app_hardware_failed_mask = 0U;
volatile uint32_t g_app_task_attempted_mask = 0U;
volatile uint32_t g_app_task_failed_mask = 0U;
volatile uint32_t g_app_control_enabled = 0U;
}

namespace app {

const char* system_state_name(SystemState state)
{
    switch (state) {
    case SystemState::booting: return "booting";
    case SystemState::ready: return "ready";
    case SystemState::initialization_failed: return "init-failed";
    case SystemState::task_failed: return "task-failed";
    case SystemState::runtime_fault: return "runtime-fault";
    }
    return "unknown";
}

void RuntimeStatus::reset()
{
    const CriticalSection lock;
    g_app_hardware_attempted_mask = 0U;
    g_app_hardware_failed_mask = 0U;
    g_app_task_attempted_mask = 0U;
    g_app_task_failed_mask = 0U;
    g_app_control_enabled = 0U;
    g_app_system_state = static_cast<uint32_t>(SystemState::booting);
}

void RuntimeStatus::publish_initialization_report(const InitializationReport& report)
{
    const CriticalSection lock;
    g_app_hardware_attempted_mask = report.attempted_mask;
    g_app_hardware_failed_mask = report.failed_mask;
}

void RuntimeStatus::record_task_result(TaskId id, bool succeeded)
{
    if (static_cast<uint8_t>(id) >= 32U) { return; }
    const CriticalSection lock;
    const uint32_t bit = uint32_t{1} << static_cast<uint8_t>(id);
    g_app_task_attempted_mask |= bit;
    if (succeeded) {
        g_app_task_failed_mask &= ~bit;
    } else {
        g_app_task_failed_mask |= bit;
    }
}

void RuntimeStatus::set_state(SystemState state)
{
    const CriticalSection lock;
    if (g_app_system_state != static_cast<uint32_t>(SystemState::runtime_fault)) {
        g_app_system_state = static_cast<uint32_t>(state);
    }
}

void RuntimeStatus::enable_control(bool enabled)
{
    const CriticalSection lock;
    g_app_control_enabled = enabled &&
        g_app_system_state != static_cast<uint32_t>(SystemState::runtime_fault) ? 1U : 0U;
}

void RuntimeStatus::enter_runtime_fault()
{
    const CriticalSection lock;
    enable_control(false);
    set_state(SystemState::runtime_fault);
}

SystemState RuntimeStatus::state() const
{
    return static_cast<SystemState>(g_app_system_state);
}

bool RuntimeStatus::control_enabled() const
{
    return g_app_control_enabled != 0U;
}

bool RuntimeStatus::hardware_ready(uint8_t id) const
{
    if (id >= 32U) {
        return false;
    }
    const uint32_t bit = uint32_t{1} << id;
    return (g_app_hardware_attempted_mask & bit) != 0U &&
           (g_app_hardware_failed_mask & bit) == 0U;
}

uint32_t RuntimeStatus::hardware_failed_mask() const
{
    return g_app_hardware_failed_mask;
}

uint32_t RuntimeStatus::task_failed_mask() const
{
    return g_app_task_failed_mask;
}

} // namespace app
