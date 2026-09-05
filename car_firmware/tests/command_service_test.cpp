#include <cstring>
#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "test_check.hpp"

namespace {
unsigned idle_waits{};
BaseType_t run_one_period()
{
    if (idle_waits++ == 0U) { fake_rtos::now += 100U; return pdFALSE; }
    throw fake_rtos::LoopDone{};
}
}

int main()
{
    fake_rtos::reset();
    bsp::BoardHardware board;
    app::ControlState control;
    app::RuntimeStatus status;
    status.reset();
    app::InitializationReport report;
    report.attempted_mask = 0xFFU;
    status.publish_initialization_report(report);
    app::ButtonEventQueue events;
    app::ButtonTask button(board, events);
    app::CommandServiceTask command(board, status, control, button, events);
    CHECK(command.start());
    const auto entry = fake_rtos::entry;
    void* const argument = fake_rtos::argument;

    // Execute the real task body with test-only notification and I/O adapters.
    auto dispatch = [&](const char* text, bool radio = false, bool periodic = false) {
        if (radio) { NRF24L01P::str_touint8(text, board.radio().received.data()); }
        else { board.command_uart().received = text; }
        fake_rtos::pending_notifications = radio ? 2U : 1U;
        idle_waits = 0U;
        fake_rtos::on_wait = periodic ? run_one_period : nullptr;
        try { entry(argument); CHECK(false); }
        catch (const fake_rtos::LoopDone&) {}
        CHECK(fake_rtos::critical_depth == 0);
    };

    dispatch("R 10 20 5 60\r\n");
    auto parameters = control.parameters();
    CHECK(parameters.difference_target == 10.0F && parameters.velocity_target == 20.0F);
    CHECK(parameters.roll_target == 5.0F && parameters.leg_height == 60.0F);
    dispatch("R 30 40 invalid 70");
    parameters = control.parameters();
    CHECK(parameters.difference_target == 10.0F && parameters.velocity_target == 20.0F);
    CHECK(parameters.leg_height == 60.0F);
    dispatch("VandD 1 2 junk");
    CHECK(control.parameters().velocity_target == 20.0F);
    dispatch("VandD\t3\t4");
    CHECK(control.parameters().difference_target == 3.0F && control.parameters().velocity_target == 4.0F);

    dispatch("rollpid -p 1.25");
    CHECK(control.parameters().roll.kp == 1.25F && control.parameters().roll.ki == -0.4F);
    dispatch("rollpid -i -0.5");
    CHECK(control.parameters().roll.ki == -0.5F);
    dispatch("anglepid -d 55");
    CHECK(control.parameters().angle.kd == 55.0F);
    dispatch("anglepid -d");
    dispatch("anglepid -d nan");
    dispatch("anglepid -d 1junk");
    CHECK(control.parameters().angle.kd == 55.0F);
    dispatch("showimu -y");
    CHECK(control.parameters().show_imu);
    dispatch("showimu invalid");
    CHECK(control.parameters().show_imu);
    dispatch("showimu -n");
    CHECK(!control.parameters().show_imu);
    dispatch("target_roll 8");
    CHECK(control.parameters().roll_target == 8.0F);
    dispatch("legheight 50");
    CHECK(control.parameters().leg_height == 50.0F);
    dispatch("legheight 60                     extra");
    CHECK(control.parameters().leg_height == 50.0F);
    dispatch("motor 100 100");
    CHECK(board.command_uart().logs.back().find("raw PWM is unavailable") != std::string::npos);
    dispatch("ping");
    CHECK(board.command_uart().logs.back().find("pong") != std::string::npos);

    dispatch("R 5 6 7 55", true);
    CHECK(control.parameters().velocity_target == 6.0F && control.parameters().leg_height == 55.0F);
    board.radio().receive_ok = false;
    dispatch("R 90 90 90 90", true);
    CHECK(control.parameters().velocity_target == 6.0F);
    control.publish_feedback({{1.0F, 2.0F, 3.0F}, 70.0F, 12.6F});
    dispatch("nrfshow -mr 0", false, true);
    CHECK(board.radio().sent.size() == 1U);
    CHECK(std::strcmp(reinterpret_cast<const char*>(board.radio().sent.back().data()),
                       " 1.0 2.0 3.0 70.0") == 0);
    dispatch("nrfshow -nn", false, true);
    CHECK(board.radio().sent.size() == 1U);

    CHECK(events.push({app::Button::Event::click, 10U}));
    CHECK(events.push({app::Button::Event::double_click, 20U}));
    CHECK(events.push({app::Button::Event::long_press, 30U}));
    dispatch("button");
    CHECK(board.command_uart().logs.back().find("click=1 double=1 long=1 drop=0") != std::string::npos);
    CHECK(fake_rtos::notifications == 0U);
}
