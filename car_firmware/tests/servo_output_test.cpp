#include "Servo.hpp"
#include "test_check.hpp"

int main() {
    TIM_HandleTypeDef timer;
    Servo servo(&timer, 1, 500, 2500, 180);
    CHECK(servo.Init() && timer.enabled);
    servo.setAngle_Smooth(40, 1000);
    servo.updateSmoothing(); CHECK(timer.compare > 500);
    servo.stop(); CHECK(!timer.enabled && timer.compare == 0);
    // A callback already queued before stop cannot resurrect the PWM compare.
    servo.updateSmoothing(); CHECK(!timer.enabled && timer.compare == 0);
    servo.setAngle_Smooth(1.188965F, 1000);
    CHECK(timer.enabled);
    servo.updateSmoothing();
    CHECK(timer.compare == 513 && servo.getCurrentAngle() == 1.188965F);
    for (unsigned i = 0; i < 30; ++i) servo.updateSmoothing();
    CHECK(timer.compare == 513 && timer.enabled);
    servo.stop(); CHECK(timer.compare == 0);
    const auto starts = timer_instance.starts;
    servo.setAngle_Smooth(18, 0);
    CHECK(timer.enabled && timer.compare == 700 && timer_instance.starts == starts);
    CHECK(fake_rtos::critical_depth == 0);
}
