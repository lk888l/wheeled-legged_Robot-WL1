#include "ServoControlTask.hpp"

#include <algorithm>
#include <cmath>
#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"

namespace app {

void ServoControlTask::run()
{
    auto& left = board_.left_servo();
    auto& right = board_.right_servo();
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const bool installing = control_.installation_mode();
        if (!status_.control_enabled() &&
            !(installing && status_.state() == SystemState::ready)) {
            left.stop();
            right.stop();
            control_.set_installation_ready(false);
            continue;
        }
        const LegTargets targets = installing ? LegTargets{} : control_.leg_targets();
        const float left_angle = LegKinematics::getMotorAngleForHeight(
            std::clamp(targets.left, 44.5F, 78.5F));
        const float right_angle = LegKinematics::getMotorAngleForHeight(
            std::clamp(targets.right, 44.5F, 78.5F));
        // Motion can preempt kinematics and enter safe mode.
        taskENTER_CRITICAL();
        if (installing == control_.installation_mode() &&
            (status_.control_enabled() || (installing && status_.state() == SystemState::ready))) {
            // Ready confirms the commanded PWM has settled, not physical feedback.
            control_.set_installation_ready(installing &&
                std::fabs(left.getCurrentAngle() - (left_angle - 10.0F)) < 0.1F &&
                std::fabs(right.getCurrentAngle() - (right_angle - 10.0F)) < 0.1F);
            left.setAngle_Smooth(left_angle - 10.0F, 1000.0F);
            right.setAngle_Smooth(right_angle - 10.0F, 1000.0F);
        }
        taskEXIT_CRITICAL();
    }
}

} // namespace app
