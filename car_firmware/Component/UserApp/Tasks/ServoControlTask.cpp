#include "ServoControlTask.hpp"

#include <algorithm>
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
        if (!status_.control_enabled()) {
            left.stop();
            right.stop();
            continue;
        }
        const LegTargets targets = control_.leg_targets();
        const float left_angle = LegKinematics::getMotorAngleForHeight(
            std::clamp(targets.left, 44.5F, 78.5F));
        const float right_angle = LegKinematics::getMotorAngleForHeight(
            std::clamp(targets.right, 44.5F, 78.5F));
        // Motion can preempt kinematics and enter safe mode.
        if (status_.control_enabled()) {
            left.setAngle_Smooth(left_angle - 10.0F, 1000.0F);
            right.setAngle_Smooth(right_angle - 10.0F, 1000.0F);
        }
    }
}

} // namespace app
