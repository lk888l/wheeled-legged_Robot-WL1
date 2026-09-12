#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "test_check.hpp"

namespace {
std::function<void()> step;
void advance() { step(); }
class ServoPeer final : public app::AppTask {
public:
    ServoPeer() : AppTask(app::task_config::servo) {}
private:
    void run() override { throw fake_rtos::LoopDone{}; }
};

struct Fixture {
    bsp::BoardHardware board;
    app::ControlState control;
    app::RuntimeStatus status;
    app::ButtonEventQueue events;
    app::ButtonTask button{board, events};
    app::CommandServiceTask command{board, status, control, button, events};
    ServoPeer servo;
    app::MotionControlTask motion{board, status, control, servo};
    TaskFunction_t command_entry{};
    void* command_argument{};

    Fixture()
    {
        fake_rtos::reset();
        status.reset();
        app::InitializationReport report;
        report.attempted_mask = 0xFFU;
        status.publish_initialization_report(report);
        status.enable_control(true);
        status.set_state(app::SystemState::ready);
        CHECK(command.start());
        command_entry = fake_rtos::entry;
        command_argument = fake_rtos::argument;
        CHECK(servo.start());
    }
    void send(const std::string& text, bool radio = true)
    {
        if (radio) { NRF24L01P::str_touint8(text.c_str(), board.radio().received.data()); }
        else { board.command_uart().received = text.c_str(); }
        fake_rtos::pending_notifications = radio ? 2U : 1U;
        try { command_entry(command_argument); CHECK(false); }
        catch (const fake_rtos::LoopDone&) {}
    }
    void run()
    {
        fake_rtos::on_delay = advance;
        fake_rtos::run_on_resume = true;
        try { (void)motion.start(); CHECK(false); }
        catch (const fake_rtos::LoopDone&) {}
        CHECK(fake_rtos::critical_depth == 0);
    }
};

void test_each_gain()
{
    // Analytic step responses of the real command -> snapshot -> task -> PWM/leg path.
    for (const std::string loop : {"anglepid", "velocitypid", "differpid", "rollpid"}) {
        for (const char term : {'p', 'i', 'd'}) {
            Fixture f;
            auto parameters = f.control.parameters();
            parameters.angle = {10, 0, 0};
            parameters.angle_kp_auto = false;
            parameters.velocity = parameters.difference = parameters.roll = {0, 0, 0};
            parameters.leg_height = 61.5F;
            f.control.set_parameters(parameters);
            f.board.imu().reading.Pitch = -BalanceCompensation::pitchBias(parameters.angle_bias, 61.5F);
            const unsigned end = loop == "anglepid" ? 520U : 560U;
            step = [&] {
                const auto tick = fake_rtos::now;
                if (tick < 510U) {
                    CHECK(f.board.wheel_motor().left == 0 && f.board.wheel_motor().right == 0);
                    if (tick <= 500U) { CHECK(!f.control.feedback().armed); }
                }
                if (tick == 510U) {
                    CHECK(f.control.feedback().armed);
                    if (loop == "anglepid") {
                        f.send("anglepid -p 0");
                        f.board.imu().reading.Pitch -= 1.0;
                    } else if (loop == "velocitypid") {
                        f.board.left_encoder().rpm = f.board.right_encoder().rpm = 1;
                    } else if (loop == "differpid") {
                        f.board.left_encoder().rpm = 1;
                        f.board.right_encoder().rpm = -1;
                    } else {
                        f.board.imu().reading.Roll = 1;
                    }
                    f.send(loop + " -" + term + " 1");
                }
                if (tick == end) {
                    CHECK(f.control.feedback().armed);
                    if (loop == "anglepid") {
                        CHECK(f.board.wheel_motor().left == 1 && f.board.wheel_motor().right == 1);
                    } else if (loop == "velocitypid") {
                        CHECK(f.board.wheel_motor().left == -10 && f.board.wheel_motor().right == -10);
                    } else if (loop == "differpid") {
                        CHECK(f.board.wheel_motor().left == -2 && f.board.wheel_motor().right == 2);
                    } else {
                        const auto legs = f.control.leg_targets();
                        CHECK(std::fabs(legs.left - 62.5F) < 0.0001F);
                        CHECK(std::fabs(legs.right - 60.5F) < 0.0001F);
                    }
                    throw fake_rtos::LoopDone{};
                }
            };
            f.run();
        }
    }
}

void test_calibration_and_recovery()
{
    Fixture f;
    f.send("anglepid -d 0");
    step = [&] {
        const auto tick = fake_rtos::now;
        if (tick == 510U) {
            CHECK(f.control.feedback().armed);
            CHECK(std::fabs(f.control.feedback().angle_kp - 70.25F) < 0.0001F);
            f.send("anglepid -p 80");
            f.send("R 0 0 0 61.5");
        }
        if (tick == 560U) {
            CHECK(f.control.feedback().angle_kp == 80);
            CHECK(std::fabs(f.control.feedback().angle_bias - 6.60252F) < 0.0001F);
            f.send("anglebias 10.5", false);
            f.board.imu().reading.Roll = 4;
        }
        if (tick == 710U) {
            const auto legs = f.control.leg_targets();
            CHECK(legs.left != legs.right);
            CHECK(std::fabs((legs.left + legs.right) / 2 - 61.5F) < 0.0001F);
            CHECK(std::fabs(f.control.feedback().angle_bias - 7.60252F) < 0.0001F);
            CHECK(f.control.feedback().angle_kp == 80);
            CHECK(f.control.parameters().angle_bias == 10.5F);
            f.send("anglepid -p nan");
            f.send("anglebias inf");
            CHECK(f.control.parameters().angle.kp == 80 && !f.control.parameters().angle_kp_auto);
            CHECK(f.control.parameters().angle_bias == 10.5F);
            f.send("anglepid -auto");
        }
        if (tick == 720U) {
            CHECK(std::fabs(f.control.feedback().angle_kp - 75.35F) < 0.0001F);
            f.board.imu().healthy = false;
        }
        if (tick == 730U) {
            CHECK(!f.control.feedback().armed && !f.control.feedback().imu_valid);
            CHECK(f.board.wheel_motor().left == 0 && f.board.wheel_motor().right == 0);
            f.board.imu().healthy = true;
            f.board.imu().reading = {0, -7.60252, 0};
        }
        if (tick > 730U && tick <= 1220U) { CHECK(!f.control.feedback().armed); }
        if (tick == 1230U) {
            CHECK(f.control.feedback().armed);
            f.board.imu().reading.Pitch = 40;
        }
        if (tick == 1240U) {
            CHECK(!f.control.feedback().armed && f.board.wheel_motor().left == 0);
            f.board.imu().reading.Pitch = std::numeric_limits<double>::quiet_NaN();
        }
        if (tick == 1270U) {
            CHECK(f.status.state() == app::SystemState::runtime_fault);
            CHECK(!f.status.control_enabled());
            CHECK(f.board.safe_stops == 1U);
            CHECK(f.control.feedback().max_sample_gap_ticks == 10U);
            CHECK(f.control.feedback().deadline_misses == 0U);
            throw fake_rtos::LoopDone{};
        }
    };
    f.run();
}
void test_remote_timeout()
{
    Fixture f;
    step = [&] {
        const auto tick = fake_rtos::now;
        if (tick == 510U) {
            CHECK(f.control.feedback().armed);
            f.send("@R 10 20 4 61.5\n", false);
        }
        if (tick == 520U) {
            CHECK(f.control.feedback().velocity_target == 20);
            CHECK(f.control.feedback().difference_target == 10);
        }
        if (tick == 900U) {
            f.send("@anglepid -p 80\n", false);
            CHECK(f.control.parameters().motion_command_tick == 510U);
        }
        if (tick == 1010U) { CHECK(!f.control.feedback().remote_timed_out); }
        if (tick == 1020U) {
            const auto feedback = f.control.feedback();
            CHECK(feedback.remote_timed_out && feedback.armed && feedback.imu_valid);
            CHECK(feedback.velocity_target == 0 && feedback.difference_target == 0 && feedback.roll_target == 0);
            CHECK(f.control.parameters().leg_height == 61.5F);
            CHECK(feedback.angle_kp == 80 && feedback.deadline_misses == 0U);
            CHECK(f.status.control_enabled());
            // Tuning and malformed commands do not extend the motion deadline.
            f.send("@R 10 invalid 4 61.5\n", false);
            CHECK(f.control.parameters().motion_command_tick == 510U);
        }
        if (tick == 1050U) {
            f.send("target_roll 6", false);
            CHECK(f.control.parameters().velocity_target == 0 && f.control.parameters().difference_target == 0);
        }
        if (tick == 1060U) {
            CHECK(!f.control.feedback().remote_timed_out && f.control.feedback().roll_target == 6);
            CHECK(f.control.feedback().velocity_target == 0);
            f.send("@R 999 -999 -99 99\n", false);
            const auto p = f.control.parameters();
            CHECK(p.difference_target == 100 && p.velocity_target == -100 && p.roll_target == -18 && p.leg_height == 78.5F);
            throw fake_rtos::LoopDone{};
        }
    };
    f.run();
}
} // namespace

int main()
{
    test_each_gain();
    test_calibration_and_recovery();
    test_remote_timeout();
}
