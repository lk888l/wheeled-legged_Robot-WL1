#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "Tasks/ServoControlTask.hpp"
#include "test_check.hpp"
#include <cmath>

namespace {
bsp::BoardHardware board;
app::ControlState control;
app::RuntimeStatus status;
app::ButtonEventQueue events;
app::ButtonTask button(board, events);
app::ServoControlTask servo(board, status, control);
app::CommandServiceTask command(board, status, control, button, events);
app::MotionControlTask motion(board, status, control, servo);
TaskFunction_t command_entry, servo_entry;
void *command_arg, *servo_arg;
unsigned writes_at_install;
void dispatch(const char* text, bool radio = false) {
    if (radio) NRF24L01P::str_touint8(text, board.radio().received.data());
    else board.command_uart().received = text;
    fake_rtos::pending_notifications = radio ? 2U : 1U;
    try { command_entry(command_arg); CHECK(false); } catch (const fake_rtos::LoopDone&) {}
}
void advance() {
    fake_rtos::pending_notifications = 1U;
    try { servo_entry(servo_arg); CHECK(false); } catch (const fake_rtos::LoopDone&) {}
    const auto now = fake_rtos::now;
    if (now == 600) {
        CHECK(control.feedback().armed);
        CHECK(board.wheel_motor().left != 0);
        // Command preempts an in-flight motion iteration after its first snapshot.
        board.left_encoder().on_read = [] {
            board.left_encoder().on_read = nullptr;
            dispatch("@install on\n");
            CHECK(board.wheel_motor().left == 0 && board.wheel_motor().right == 0);
            writes_at_install = board.wheel_motor().writes;
        };
    }
    if (now >= 610 && now <= 1700) {
        CHECK(control.installation_mode() && !status.control_enabled());
        CHECK(!control.feedback().armed);
        CHECK(board.wheel_motor().writes == writes_at_install);
        CHECK(board.wheel_motor().left == 0 && board.wheel_motor().right == 0);
        CHECK(control.leg_targets().left == 44.5F && control.leg_targets().right == 44.5F);
        CHECK(control.parameters().leg_height == 44.5F);
    }
    if (now == 700) {
        CHECK(control.installation_ready());
        CHECK(std::abs(board.left_servo().angle - 1.188965F) < 0.01F);
        CHECK(board.left_servo().angle == board.right_servo().angle);
        dispatch("R 99 99 18 78.5", true);
        dispatch("legheight 78.5"); dispatch("target_roll 18"); dispatch("VandD 80 80");
        dispatch("control on"); dispatch("control off"); dispatch("install on");
        CHECK(control.parameters().velocity_target == 0 && control.parameters().roll_target == 0);
        board.imu().healthy = false; // Assembly must remain independent of attitude/IMU.
    }
    if (now == 1600) {
        dispatch("install status");
        CHECK(board.command_uart().logs.back().find("active=1 ready=1") != std::string::npos);
    }
    if (now == 1700) {
        dispatch("install off");
        CHECK(!control.installation_mode() && !status.control_enabled());
        CHECK(board.left_servo().stops > 0 && board.right_servo().stops > 0);
        board.imu().healthy = true;
    }
    if (now == 1800) dispatch("control on");
    if (now > 1700 && now < 2290) CHECK(!control.feedback().armed);
    if (now == 2400) {
        CHECK(control.feedback().armed && board.wheel_motor().left != 0);
        status.enter_runtime_fault();
        dispatch("install on");
        CHECK(!control.installation_mode());
        throw fake_rtos::LoopDone{};
    }
}
}
int main() {
    fake_rtos::reset();
    status.reset();
    app::InitializationReport report; report.attempted_mask = 0xFFU;
    status.publish_initialization_report(report);
    status.set_state(app::SystemState::ready); status.enable_control(true);
    board.imu().reading.Pitch = -8.5;
    CHECK(servo.start()); servo_entry = fake_rtos::entry; servo_arg = fake_rtos::argument;
    CHECK(command.start()); command_entry = fake_rtos::entry; command_arg = fake_rtos::argument;
    dispatch("install maybe"); CHECK(!control.installation_mode());
    CHECK(motion.start());
    fake_rtos::on_delay = advance;
    try { fake_rtos::entry(fake_rtos::argument); CHECK(false); } catch (const fake_rtos::LoopDone&) {}
    CHECK(fake_rtos::critical_depth == 0);
    std::puts("PASS: installation preemption, PWM hold, command isolation, exit and fresh startup gate");
}
