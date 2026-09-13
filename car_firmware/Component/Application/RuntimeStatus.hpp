#pragma once

#include <cstdint>

#include "AppTask.hpp"
#include "InitializationReport.hpp"

namespace app {

enum class SystemState : uint32_t {
    booting = 1U,
    ready = 2U,
    initialization_failed = 3U,
    task_failed = 4U,
    runtime_fault = 5U,
};

[[nodiscard]] const char* system_state_name(SystemState state);

/**
 * Runtime status for this single-core MCU. Short critical sections serialize
 * bootstrap transitions with motion faults; a runtime fault stays latched.
 * All fields are naturally aligned 32-bit words and are also exported with C
 * linkage so they can be inspected through ST-Link without a serial adapter.
 */
class RuntimeStatus {
public:
    void reset();
    void publish_initialization_report(const InitializationReport& report);
    void record_task_result(TaskId id, bool succeeded);
    void set_state(SystemState state);
    void enable_control(bool enabled);
    void enter_runtime_fault();

    [[nodiscard]] SystemState state() const;
    [[nodiscard]] bool control_enabled() const;
    [[nodiscard]] bool hardware_ready(uint8_t id) const;
    [[nodiscard]] uint32_t hardware_failed_mask() const;
    [[nodiscard]] uint32_t task_failed_mask() const;
};

} // namespace app

extern "C" {
extern volatile uint32_t g_app_system_state;
extern volatile uint32_t g_app_hardware_attempted_mask;
extern volatile uint32_t g_app_hardware_failed_mask;
extern volatile uint32_t g_app_task_attempted_mask;
extern volatile uint32_t g_app_task_failed_mask;
extern volatile uint32_t g_app_control_enabled;
}
