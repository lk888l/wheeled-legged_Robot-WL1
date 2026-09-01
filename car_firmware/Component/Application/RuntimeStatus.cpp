#include "RuntimeStatus.hpp"

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

void RuntimeStatus::reset()
{
    g_app_hardware_attempted_mask = 0U;
    g_app_hardware_failed_mask = 0U;
    g_app_task_attempted_mask = 0U;
    g_app_task_failed_mask = 0U;
    g_app_control_enabled = 0U;
    g_app_system_state = static_cast<uint32_t>(SystemState::booting);
}

void RuntimeStatus::publish_initialization_report(const InitializationReport& report)
{
    g_app_hardware_attempted_mask = report.attempted_mask;
    g_app_hardware_failed_mask = report.failed_mask;
}

void RuntimeStatus::record_task_result(TaskId id, bool succeeded)
{
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
    g_app_system_state = static_cast<uint32_t>(state);
}

void RuntimeStatus::enable_control(bool enabled)
{
    g_app_control_enabled = enabled ? 1U : 0U;
}

void RuntimeStatus::enter_runtime_fault()
{
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
