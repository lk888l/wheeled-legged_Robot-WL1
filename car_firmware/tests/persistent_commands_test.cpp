#include <string>
#include "BoardHardware.hpp"
#include "PersistentFlash.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "test_check.hpp"

int main()
{
    fake_rtos::reset();
    fake_flash::reset();
    bsp::BoardHardware board;
    app::RuntimeStatus status;
    status.reset();
    status.set_state(app::SystemState::ready);
    app::InitializationReport report;
    report.attempted_mask = 0xFF;
    status.publish_initialization_report(report);
    app::ControlState control;
    app::ButtonEventQueue events;
    app::ButtonTask button(board, events);
    app::CommandServiceTask command(board, status, control, button, events);
    CHECK(!command.load_parameters());
    CHECK(command.start());
    const auto entry = fake_rtos::entry;
    void* argument = fake_rtos::argument;
    const auto send = [&](const char* text, bool radio = false) {
        if (radio) NRF24L01P::str_touint8(text, board.radio().received.data());
        else board.command_uart().received = text;
        fake_rtos::pending_notifications = radio ? 2U : 1U;
        try { entry(argument); CHECK(false); } catch (const fake_rtos::LoopDone&) {}
        CHECK(fake_rtos::critical_depth == 0);
    };
    const auto last_contains = [&](const char* text) {
        CHECK(board.command_uart().logs.back().find(text) != std::string::npos);
    };
    send("@anglepid -p 80\n");
    send("@anglebias 10.5\n");
    send("velocitypid -i 0.007", true);
    send("differpid -d 0.02", true);
    send("@legpid -d 0.03\n");
    send("@deadzone 72\n");
    send("@R 5 6 2 61.5\n");
    CHECK(control.parameters().angle.kp == 80 && control.parameters().angle_kp_auto);
    CHECK(control.parameters().roll.kd == 0.03F);
    CHECK(control.parameters().motor_deadzone == 72U);

    app::ControlFeedback moving;
    moving.armed = true;
    moving.left_pwm = 20;
    control.publish_feedback(moving);
    status.enable_control(true);
    send("@save\n");
    last_contains("save: ok");
    CHECK(fake_flash::device.writes == 21 && fake_flash::device.yields == 21);
    CHECK(control.feedback().armed && control.feedback().left_pwm == 20);
    CHECK(status.control_enabled() && !control.consume_storage_reset());
    send("@save recycle\n");
    last_contains("save: busy");
    CHECK(fake_flash::device.writes == 21 && !control.storage_busy());
    control.publish_feedback({});
    status.enable_control(true);
    send("@control off\n");
    CHECK(!status.control_enabled());
    send("@velocitypid -p 0.06\n");
    fake_flash::device.on_write = [&] { CHECK(control.storage_busy() && !control.storage_blocks_control()); };
    send("@sa");
    CHECK(fake_flash::device.writes == 21); // Split BLE packet cannot trigger early save.
    send("ve\n");
    last_contains("save: ok");
    CHECK(!control.storage_busy() && control.consume_storage_reset());
    const auto persisted = app::MotionPersistence::snapshot(control.parameters());
    const auto writes = fake_flash::device.writes;
    send("save all", true);
    last_contains("save: unchanged");
    CHECK(fake_flash::device.writes == writes);
    send("@save nonsense\n");
    last_contains("save: usage");
    CHECK(fake_flash::device.writes == writes);

    // A fresh owner reconstructs the saved whitelist and leaves watchdog/drive state fresh.
    app::ControlState restored;
    app::MotionPersistence restore(restored);
    CHECK(restore.load());
    CHECK(MotionSettings::sameParameters(persisted, app::MotionPersistence::snapshot(restored.parameters())));
    CHECK(restored.parameters().velocity_target == 0 && restored.parameters().difference_target == 0);
    CHECK(!restored.parameters().motion_command_received && restored.parameters().motion_command_tick == 0);
    CHECK(!restored.parameters().show_imu && !restored.parameters().show_rpm);
    CHECK(restored.parameters().motor_deadzone == 72U);
    CHECK(restored.leg_targets().left == 61.5F && restored.leg_targets().right == 61.5F);
    CHECK(!restore.unsaved());

    send("@anglepid -manual 80\n");
    CHECK(!control.parameters().angle_kp_auto);
    send("@save\n");
    last_contains("save: ok"); // Same floats, but changed mode must be persisted.
    CHECK(restore.load() && !restored.parameters().angle_kp_auto);
    send("@anglepid -auto\n");
    CHECK(control.parameters().angle_kp_auto && control.parameters().angle.kp == 80);
    fake_flash::device.fail_write = true;
    send("@anglebias 12\n");
    send("@save\n");
    last_contains("save: flash error");
    CHECK(!control.storage_busy() && control.parameters().angle_bias == 12);
    CHECK(restore.load() && restored.parameters().angle_bias == 10.5F);
    send("@control on\n");
    CHECK(status.control_enabled());
    status.enter_runtime_fault();
    send("@control on\n");
    CHECK(!status.control_enabled());
    last_contains("rejected");
    fake_flash::device.fail_write = false;
    fake_flash::device.on_write = {};
    // Fill the remaining slots, then request a normal save with active PWM.
    control.publish_feedback(moving);
    for (unsigned i = 0; i < 8; ++i) {
        const auto before_tuning = fake_flash::device.words;
        send((std::string("@anglebias ") + std::to_string(11 + i) + "\n").c_str());
        CHECK(fake_flash::device.words == before_tuning); // Tuning changes RAM only.
        send("@save\n");
    }
    last_contains("save: full");
    CHECK(fake_flash::device.erases == 0U && control.feedback().armed);
    CHECK(!control.storage_busy() && !control.storage_blocks_control());
    CHECK(!control.consume_storage_reset());
}
