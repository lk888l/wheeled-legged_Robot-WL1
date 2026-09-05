#include "cpp_Interface.h"

#include "BoardHardware.hpp"
#include "ButtonEventQueue.hpp"
#include "ControlState.hpp"
#include "InitializationReport.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "Tasks/HeartbeatTask.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "Tasks/ServoControlTask.hpp"
#include "Tasks/TaskConfig.hpp"

namespace {

bool start_task(app::AppTask& task, bsp::BoardHardware& board, app::RuntimeStatus& status)
{
    const bool started = task.start();
    status.record_task_result(task.id(), started);
    board.command_uart().print("[task][{}] {}\n", started ? " OK " : "FAIL", task.name());
    vTaskDelay(pdMS_TO_TICKS(3U));
    return started;
}

} // namespace

// Composition root only. Business loops and their dependencies live in Tasks/.
// Called once by the CMSIS bootstrap after the FreeRTOS scheduler is running.
void CPP_Main()
{
    // 1. Assemble persistent modules and inject task dependencies.
    static bsp::BoardHardware board;
    static app::RuntimeStatus status;
    static app::ControlState control;
    static app::ButtonEventQueue button_events;
    static app::HeartbeatTask heartbeat(board, status);
    static app::ServoControlTask servo(board, status, control);
    static app::MotionControlTask motion(board, status, control, servo);
    static app::ButtonTask button(board, button_events);
    static app::CommandServiceTask command(board, status, control, button, button_events);

    // 2. Keep outputs safe and start independent startup diagnostics.
    status.reset();
    board.force_safe_outputs();
    board.command_uart().print("[app] WL1 startup begin\n");
    const bool heartbeat_started = start_task(heartbeat, board, status);

    // 3. Initialize each hardware module explicitly, in dependency order.
    // record_init only records/logs results; every driver call is visible here.
    app::InitializationReport report;
    const auto record_init = [&](bsp::HardwareModuleId id, bool succeeded) {
        report.record(bsp::module_id(id), succeeded);
        board.command_uart().print("[init][{}] {}\n",
            report.succeeded(bsp::module_id(id)) ? " OK " : "FAIL", bsp::module_name(id));
        // Preserve pacing for the existing asynchronous UART buffer capacity.
        vTaskDelay(pdMS_TO_TICKS(3U));
    };
    using bsp::HardwareModuleId;
    record_init(HardwareModuleId::command_uart,  board.initialize_command_uart());
    record_init(HardwareModuleId::imu,           board.initialize_imu());
    record_init(HardwareModuleId::left_encoder,  board.initialize_left_encoder());
    record_init(HardwareModuleId::right_encoder, board.initialize_right_encoder());
    record_init(HardwareModuleId::wheel_motor,   board.initialize_wheel_motor());
    record_init(HardwareModuleId::left_servo,    board.initialize_left_servo());
    record_init(HardwareModuleId::right_servo,   board.initialize_right_servo());
    record_init(HardwareModuleId::radio,         board.initialize_radio());
    status.publish_initialization_report(report);
    const bool hardware_ready = report.all_succeeded(bsp::kRequiredHardwareMask);

    // 4. Retain command diagnostics whenever either command channel is usable.
    const bool command_channel_available =
        status.hardware_ready(bsp::module_id(bsp::HardwareModuleId::command_uart)) ||
        status.hardware_ready(bsp::module_id(bsp::HardwareModuleId::radio));
    bool command_started = true;
    if (command_channel_available) {
        command_started = start_task(command, board, status);
    } else {
        board.command_uart().print("[task][SKIP] CommandService: no command channel\n");
    }

    // 5. Start control only after all required hardware and tasks succeed.
    bool servo_started = false;
    bool motion_started = false;
    const bool may_start_control = hardware_ready && heartbeat_started && command_started;
    if (may_start_control) {
        servo_started = start_task(servo, board, status);
        if (servo_started) {
            status.enable_control(true);
            motion_started = start_task(motion, board, status);
        }
    }
    if (!may_start_control || !servo_started || !motion_started) {
        status.enable_control(false);
        board.force_safe_outputs();
    }

    // 6. The optional button uses static storage and starts after required tasks.
    // Its failure is observable in bit 4 but cannot disable the control loop.
    (void)start_task(button, board, status);
    if ((status.task_failed_mask() & app::task_config::required_task_mask) != 0U) {
        status.set_state(app::SystemState::task_failed);
    } else if (!hardware_ready) {
        status.set_state(app::SystemState::initialization_failed);
    } else if (status.control_enabled()) {
        status.set_state(app::SystemState::ready);
    } else {
        status.set_state(app::SystemState::task_failed);
    }

    board.command_uart().print("[app] state={} control={} hw_fail={} task_fail={}\n",
        app::system_state_name(status.state()), status.control_enabled() ? "on" : "off",
        status.hardware_failed_mask(), status.task_failed_mask());
}
