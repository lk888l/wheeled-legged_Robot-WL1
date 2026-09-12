#include <cmath>
#include <vector>

#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/ButtonTask.hpp"
#include "Tasks/CommandServiceTask.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "test_check.hpp"

namespace {

class ServoPeer final : public app::AppTask {
public:
    ServoPeer() : AppTask(app::task_config::servo) {}
private:
    void run() override { throw fake_rtos::LoopDone{}; }
};

struct Sample {
    TickType_t tick;
    app::ControlParameters parameters;
    app::LegTargets legs;
    app::ControlFeedback feedback;
    int left_pwm;
    int right_pwm;
};

class Harness {
public:
    bsp::BoardHardware board;
    app::ControlState control;
    app::RuntimeStatus status;
    app::ButtonEventQueue events;
    app::ButtonTask button{board, events};
    ServoPeer servo;
    app::CommandServiceTask command{board, status, control, button, events};
    app::MotionControlTask motion{board, status, control, servo};
    TaskFunction_t command_entry{};
    void* command_argument{};
    std::vector<Sample> samples;

    void dispatch(const char* text, bool radio = false)
    {
        if (radio) { NRF24L01P::str_touint8(text, board.radio().received.data()); }
        else { board.command_uart().received = text; }
        fake_rtos::pending_notifications = radio ? 2U : 1U;
        try { command_entry(command_argument); CHECK(false); }
        catch (const fake_rtos::LoopDone&) {}
        CHECK(fake_rtos::critical_depth == 0);
    }

    void advance()
    {
        // The previous 10 ms iteration has completed and published feedback.
        // Commands below arrive before this iteration snapshots its parameters.
        if (fake_rtos::now > 10U) {
            samples.push_back({fake_rtos::now - 10U, control.parameters(), control.leg_targets(),
                              control.feedback(), board.wheel_motor().left, board.wheel_motor().right});
        }
        switch (fake_rtos::now) {
        case 20U: dispatch("anglebias 10.5"); break;
        case 30U: dispatch("anglebias 11.5", true); break;
        case 50U: dispatch("R 0 0 0 60", true); break;
        case 60U: dispatch("anglebias 12.5", true); break;
        case 100U: dispatch("R 0 0 15 45", true); break;
        case 150U: dispatch("R 0 0 -15 45", true); break;
        case 200U: dispatch("R 0 0 0 78.5", true); break;
        case 250U: dispatch("R 0 0 0 0", true); break;
        case 300U: dispatch("R 0 0 0 100", true); break;
        case 350U: dispatch("R 0 0 0 60", true); break;
        case 710U: throw fake_rtos::LoopDone{};
        default: break;
        }
    }

    const Sample& at(TickType_t tick) const
    {
        CHECK(tick >= 10U && tick % 10U == 0U);
        const auto& sample = samples.at(tick / 10U - 1U);
        CHECK(sample.tick == tick);
        return sample;
    }
};

Harness* active{};
void advance() { active->advance(); }

double reference_bias(double baseline, double height)
{
    // Deliberately use the unfactored calibration in double precision rather
    // than the production helper, so a changed coefficient cannot bless itself.
    const auto curve = [](double h) { return 0.01026 * h * h - 1.258 * h + 48.24; };
    return baseline + curve(height) - curve(44.5);
}

} // namespace

int main()
{
    fake_rtos::reset();
    Harness harness;
    active = &harness;
    harness.status.reset();
    app::InitializationReport report;
    report.attempted_mask = 0xFFU;
    harness.status.publish_initialization_report(report);
    harness.status.enable_control(true);
    harness.status.set_state(app::SystemState::ready);
    CHECK(harness.control.parameters().angle_bias ==
          BalanceCompensation::default_minimum_bias_degrees);
    CHECK(harness.control.leg_targets().left == 44.5F);
    CHECK(harness.control.leg_targets().right == 44.5F);

    // A neutral stationary pose makes the expected PWM independently calculable.
    // Disable derivative/integral terms so output changes expose bias changes.
    harness.board.imu().reading = {
        0.0, -BalanceCompensation::default_minimum_bias_degrees, 0.0};
    auto parameters = harness.control.parameters();
    parameters.angle.ki = parameters.angle.kd = 0.0F;
    parameters.roll = {0.0F, 0.0F, 0.0F};
    harness.control.set_parameters(parameters);
    CHECK(harness.servo.start());
    CHECK(harness.command.start());
    harness.command_entry = fake_rtos::entry;
    harness.command_argument = fake_rtos::argument;
    CHECK(harness.motion.start());
    fake_rtos::on_delay = advance;
    try { fake_rtos::entry(fake_rtos::argument); CHECK(false); }
    catch (const fake_rtos::LoopDone&) {}

    CHECK(harness.samples.size() == 70U);
    CHECK(harness.board.imu().samples.size() == 70U);
    std::size_t armed_samples = 0U;
    for (const auto& sample : harness.samples) {
        if (sample.feedback.armed) { ++armed_samples; }
    }
    CHECK(harness.board.wheel_motor().writes == 2U * armed_samples);
    CHECK(fake_rtos::notifications == 14U);
    CHECK(fake_rtos::critical_depth == 0 && harness.board.safe_stops == 0U);
    CHECK(harness.status.control_enabled());

    for (const auto& sample : harness.samples) {
        const float baseline = sample.tick < 20U
                             ? BalanceCompensation::default_minimum_bias_degrees
                             : sample.tick < 30U ? 10.5F
                             : sample.tick < 60U ? 11.5F : 12.5F;
        CHECK(sample.parameters.angle_bias == baseline);
        CHECK(sample.legs.left >= 44.5F && sample.legs.left <= 78.5F);
        CHECK(sample.legs.right >= 44.5F && sample.legs.right <= 78.5F);
        const double height = (double(sample.legs.left) + sample.legs.right) / 2.0;
        const double bias = reference_bias(baseline, height);
        const double kp = 0.3 * height + 56.9;
        CHECK(std::abs(sample.feedback.angle_bias - bias) < 0.00002);
        CHECK(std::abs(sample.feedback.angle_kp - kp) < 0.00002);
        const int pwm = static_cast<int>(std::round(
            kp * (BalanceCompensation::default_minimum_bias_degrees - bias)));
        if (sample.feedback.armed) {
            CHECK(sample.left_pwm == pwm && sample.right_pwm == pwm);
        } else {
            CHECK(sample.left_pwm == 0 && sample.right_pwm == 0);
        }
    }

    // Startup uses the real minimum height, before the first 50 ms servo update.
    CHECK(harness.at(10U).feedback.angle_bias ==
          BalanceCompensation::default_minimum_bias_degrees);
    CHECK(harness.at(10U).left_pwm == 0);
    CHECK(harness.at(20U).left_pwm == 0);
    CHECK(harness.at(30U).left_pwm == 0);
    CHECK(harness.at(50U).legs.left == 60.0F && harness.at(50U).legs.right == 60.0F);
    CHECK(harness.at(50U).feedback.angle_bias != harness.at(40U).feedback.angle_bias);
    CHECK(harness.at(60U).feedback.angle_bias != harness.at(50U).feedback.angle_bias);
    CHECK(!harness.at(490U).feedback.armed);
    CHECK(harness.at(500U).feedback.armed);

    // Roll compensation clips one leg: both bounded targets must contribute in
    // this same iteration. Reversing roll exchanges legs without changing bias.
    const auto& positive = harness.at(100U);
    const auto& negative = harness.at(150U);
    CHECK(positive.legs.left == 45.0F && positive.legs.right == 45.0F);
    CHECK(negative.legs.left == 45.0F && negative.legs.right == 45.0F);
    CHECK(positive.feedback.angle_bias == negative.feedback.angle_bias);
    CHECK(positive.feedback.angle_kp == negative.feedback.angle_kp);
    CHECK(positive.left_pwm == negative.left_pwm);
    CHECK(harness.at(200U).legs.left == 78.5F && harness.at(200U).legs.right == 78.5F);
    CHECK(harness.at(250U).legs.left == 44.5F && harness.at(250U).legs.right == 44.5F);
    CHECK(harness.at(250U).feedback.angle_bias == 12.5F);
    CHECK(harness.at(300U).legs.left == 78.5F && harness.at(300U).legs.right == 78.5F);
    CHECK(harness.at(300U).feedback.angle_bias == harness.at(200U).feedback.angle_bias);
    for (TickType_t tick = 350U; tick <= 700U; tick += 10U) {
        CHECK(harness.at(tick).legs.left == 60.0F && harness.at(tick).legs.right == 60.0F);
        CHECK(harness.at(tick).feedback.angle_bias == harness.at(60U).feedback.angle_bias);
    }
}
