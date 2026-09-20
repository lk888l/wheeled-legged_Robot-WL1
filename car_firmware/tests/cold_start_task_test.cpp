#include <cmath>
#include <functional>
#include <vector>
#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "test_check.hpp"

using app::ColdStartPhase;
using app::ColdStartAbort;

namespace {
std::function<void()> step;
void advance() { step(); }
class ServoPeer final : public app::AppTask {
public:
    ServoPeer() : AppTask(app::task_config::servo) {}
private:
    void run() override { throw fake_rtos::LoopDone{}; }
};
struct Sample {
    unsigned tick;
    app::ControlFeedback feedback;
    app::LegTargets legs;
    int left, right;
    unsigned deadzone;
};
struct Fixture {
    bsp::BoardHardware board;
    app::ControlState control;
    app::RuntimeStatus status;
    ServoPeer servo;
    // Exercise the production default: no constructor override.
    app::MotionControlTask motion{board, status, control, servo};
    std::vector<Sample> samples;

    explicit Fixture(bool request_launch = true)
    {
        fake_rtos::reset();
        status.reset();
        status.set_state(app::SystemState::ready);
        status.enable_control(true);
        if (request_launch) { control.request_cold_start(); }
        // This supported pose fails the old compensated +/-8 degree gate.
        board.imu().reading = {4, 0, 0};
        auto p = control.parameters();
        p.motor_deadzone = 100;
        p.leg_height = 50;
        p.roll_target = -4;
        control.set_parameters(p);
        CHECK(servo.start());
    }
    void record()
    {
        if (fake_rtos::now <= 10) { return; }
        samples.push_back({fake_rtos::now - 10, control.feedback(), control.leg_targets(),
            board.wheel_motor().left, board.wheel_motor().right, board.wheel_motor().a_deadzone});
    }
    void run()
    {
        fake_rtos::on_delay = advance;
        fake_rtos::run_on_resume = true;
        try { (void)motion.start(); CHECK(false); } catch (const fake_rtos::LoopDone&) {}
        CHECK(fake_rtos::critical_depth == 0);
    }
};

void ordinary_startup(bool balanced)
{
    Fixture f(false);
    auto p = f.control.parameters();
    p.roll_target = 0;
    f.control.set_parameters(p);
    // The supported pose satisfies cold-start limits but fails ordinary arming.
    f.board.imu().reading = {0, balanced ? -BalanceCompensation::pitchBias(p.angle_bias, p.leg_height) : 12.0F, 0};
    step = [&] {
        if (fake_rtos::now > 10) {
            const auto feedback = f.control.feedback();
            const auto legs = f.control.leg_targets();
            CHECK(feedback.cold_start_phase == ColdStartPhase::complete);
            CHECK(feedback.cold_start_abort == ColdStartAbort::none);
            CHECK(feedback.cold_start_distance_mm == 0 && feedback.velocity_target == 0);
            CHECK(feedback.armed == (balanced && fake_rtos::now > 500));
            if (fake_rtos::now > 50) { CHECK(legs.left == p.leg_height && legs.right == p.leg_height); }
            CHECK(f.status.control_enabled());
        }
        if (fake_rtos::now > 12000) { throw fake_rtos::LoopDone{}; }
    };
    f.run();
}

void complete_launch()
{
    Fixture f;
    unsigned handover_tick = 0, complete_tick = 0;
    step = [&] {
        f.record();
        const auto feedback = f.control.feedback();
        // Encoder movement follows the applied speed request. Waiting/raising
        // wheels are deliberately spun to ensure those counts are discarded.
        const bool launching = feedback.cold_start_phase == ColdStartPhase::ramping ||
                               feedback.cold_start_phase == ColdStartPhase::driving;
        f.board.left_encoder().rpm = f.board.right_encoder().rpm = launching
            ? feedback.velocity_target : feedback.armed ? 0 : 300;
        if (feedback.cold_start_phase == ColdStartPhase::handing_over && handover_tick == 0) {
            handover_tick = feedback.sample_tick;
        }
        if (feedback.cold_start_phase == ColdStartPhase::complete && complete_tick == 0) {
            complete_tick = feedback.sample_tick;
        }
        if (fake_rtos::now > 8200) { throw fake_rtos::LoopDone{}; }
    };
    f.run();
    CHECK(handover_tick > 6000 && handover_tick < 6500);
    CHECK(complete_tick == handover_tick + 1500);
    CHECK(f.status.control_enabled() && f.control.feedback().armed);
    CHECK(f.board.safe_stops == 0);
    const auto p = f.control.parameters();
    CHECK(p.leg_height == 50 && p.roll_target == -4 && p.velocity_target == 0 && p.motor_deadzone == 100);
    bool saw_full_speed = false, saw_ramped_pwm = false, saw_restored_roll = false;
    float last_height = 44.5F, last_gain = 0;
    for (const auto& s : f.samples) {
        const auto phase = s.feedback.cold_start_phase;
        if (s.tick < 4000) {
            CHECK(!s.feedback.armed && s.left == 0 && s.right == 0);
            CHECK(s.feedback.cold_start_distance_mm == 0);
        } else {
            CHECK(s.feedback.armed); // No 500 ms output gap at handover/completion.
        }
        if (s.tick < handover_tick) {
            CHECK(s.legs.left == s.legs.right);
            CHECK(s.legs.left >= last_height && s.legs.left <= 69.5F);
            CHECK(s.legs.left - last_height < 0.63F);
            CHECK(s.feedback.roll_target == 0 && s.feedback.difference_target == 0);
            last_height = s.legs.left;
        }
        if (phase == ColdStartPhase::ramping) {
            CHECK(s.feedback.cold_start_gain >= last_gain);
            CHECK(std::abs(s.left) <= std::lround(1000 * s.feedback.cold_start_gain));
            CHECK(s.deadzone == unsigned(std::lround(100 * s.feedback.cold_start_gain)));
            CHECK(s.feedback.velocity_target <= 0 && s.feedback.velocity_target > -30);
            last_gain = s.feedback.cold_start_gain;
            saw_ramped_pwm |= s.left != 0 && s.feedback.cold_start_gain < 0.5F;
        }
        if (phase == ColdStartPhase::driving) {
            CHECK(s.feedback.velocity_target == -30 && s.legs.left == 69.5F);
            saw_full_speed = true;
        }
        if (s.tick == handover_tick) {
            CHECK(s.feedback.cold_start_distance_mm >= 120 && s.feedback.cold_start_distance_mm < 124);
        }
        if (s.tick >= handover_tick + 300) { CHECK(s.feedback.velocity_target == 0); }
        if (s.tick >= complete_tick) {
            CHECK(s.feedback.cold_start_phase == ColdStartPhase::complete);
            CHECK(s.feedback.roll_target == -4);
            saw_restored_roll |= s.legs.left != s.legs.right;
        }
    }
    CHECK(saw_full_speed && saw_ramped_pwm && saw_restored_roll);
}

void stopped_launch(unsigned stop_tick, bool atomic_restart, bool installation = false)
{
    Fixture f;
    step = [&] {
        f.record();
        if (fake_rtos::now == stop_tick) {
            auto stop = [&] {
                f.status.enable_control(false);
                f.control.request_control_reset();
                f.board.force_safe_outputs();
                if (installation) { f.control.set_installation_mode(true); }
                if (atomic_restart) { f.status.enable_control(true); }
                f.board.imu().reading = {0, -9.5, 0};
            };
            if (atomic_restart) {
                f.board.left_encoder().on_read = [&, stop] {
                    const auto apply_stop = stop;
                    f.board.left_encoder().on_read = nullptr;
                    apply_stop();
                };
            } else { stop(); }
        }
        if (fake_rtos::now > stop_tick && fake_rtos::now < stop_tick + 500) {
            CHECK(!f.control.feedback().armed && f.board.wheel_motor().left == 0);
        }
        if (fake_rtos::now == stop_tick + 1000) { throw fake_rtos::LoopDone{}; }
    };
    f.run();
    CHECK(f.control.feedback().cold_start_phase == ColdStartPhase::aborted);
    CHECK(f.control.feedback().cold_start_abort == ColdStartAbort::interrupted);
    CHECK(f.control.feedback().armed == atomic_restart);
    CHECK(f.control.feedback().velocity_target == 0); // No second automatic departure.
}

void failed_launch(bool imu_failure, bool fall)
{
    Fixture f;
    step = [&] {
        f.record();
        if (fake_rtos::now == 4500) {
            if (imu_failure) { f.board.imu().healthy = false; }
            if (fall) { f.board.imu().reading.Roll = 31; }
        }
        if (fake_rtos::now > 10200) { throw fake_rtos::LoopDone{}; }
    };
    f.run();
    CHECK(!f.status.control_enabled() && !f.control.feedback().armed);
    CHECK(f.board.wheel_motor().left == 0 && f.board.wheel_motor().right == 0);
    CHECK(f.control.feedback().cold_start_phase == ColdStartPhase::aborted);
    CHECK(f.control.feedback().cold_start_abort == (imu_failure ? ColdStartAbort::imu :
                                                   fall ? ColdStartAbort::attitude : ColdStartAbort::timeout));
    CHECK(f.control.feedback().cold_start_distance_mm == 0);
    if (imu_failure) { CHECK(f.status.state() == app::SystemState::runtime_fault); }
}
} // namespace

int main()
{
    ordinary_startup(false);
    ordinary_startup(true);
    complete_launch();
    stopped_launch(1000, false);
    stopped_launch(4500, true);
    stopped_launch(4500, false, true);
    failed_launch(false, false);
    failed_launch(true, false);
    failed_launch(false, true);
}
