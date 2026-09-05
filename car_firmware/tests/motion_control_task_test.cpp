#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "Tasks/MotionControlTask.hpp"
#include "test_check.hpp"

namespace {
bsp::BoardHardware board;
class ServoPeer final : public app::AppTask {
public:
    ServoPeer() : AppTask(app::task_config::servo) {}
private:
    void run() override { throw fake_rtos::LoopDone{}; }
};

void advance()
{
    if (fake_rtos::now >= 170U) { throw fake_rtos::LoopDone{}; }
    // Ten good samples, then three failed samples; later recovery must not
    // silently re-enable outputs after the runtime fault has latched.
    board.imu().healthy = fake_rtos::now < 110U || fake_rtos::now >= 140U;
}
}

int main()
{
    fake_rtos::reset();
    app::ControlState control;
    app::RuntimeStatus status;
    status.reset();
    status.enable_control(true);
    status.set_state(app::SystemState::ready);
    ServoPeer servo;
    CHECK(servo.start());
    app::MotionControlTask motion(board, status, control, servo);
    fake_rtos::run_on_resume = true;
    fake_rtos::on_delay = advance;
    try { (void)motion.start(); CHECK(false); }
    catch (const fake_rtos::LoopDone&) {}
    CHECK(board.imu().samples.size() == 13U);
    for (size_t i = 0U; i < board.imu().samples.size(); ++i) {
        CHECK(board.imu().samples[i] == (i + 1U) * 10U);
    }
    CHECK(fake_rtos::notifications == 2U); // 50 ms and 100 ms servo updates.
    CHECK(board.wheel_motor().writes == 20U); // Only the ten valid IMU samples.
    CHECK(board.wheel_motor().left == 0 && board.wheel_motor().right == 0);
    CHECK(status.state() == app::SystemState::runtime_fault && !status.control_enabled());
    CHECK(board.safe_stops == 1U && fake_rtos::critical_depth == 0);
    const auto legs = control.leg_targets();
    CHECK(legs.left == 44.5F && legs.right == 44.5F);
}
