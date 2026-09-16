#include "ControlState.hpp"
#include "CriticalSection.hpp"

namespace app {

ControlParameters ControlState::parameters() const
{
    const CriticalSection lock;
    return parameters_;
}

void ControlState::set_parameters(const ControlParameters& parameters)
{
    const CriticalSection lock;
    parameters_ = parameters;
    if (installation_mode_) {
        parameters_.velocity_target = parameters_.difference_target = parameters_.roll_target = 0.0F;
        parameters_.leg_height = BalanceCompensation::minimum_leg_height_mm;
        parameters_.motion_command_received = false;
    }
}

ControlFeedback ControlState::feedback() const
{
    const CriticalSection lock;
    return feedback_;
}

void ControlState::publish_feedback(const ControlFeedback& feedback)
{
    const CriticalSection lock;
    feedback_ = feedback;
    if (installation_mode_) {
        feedback_.armed = false;
        feedback_.left_pwm = feedback_.right_pwm = 0;
        feedback_.velocity_target = feedback_.difference_target = feedback_.roll_target = 0.0F;
    }
}

LegTargets ControlState::leg_targets() const
{
    const CriticalSection lock;
    return leg_targets_;
}

void ControlState::publish_leg_targets(LegTargets targets)
{
    const CriticalSection lock;
    leg_targets_ = installation_mode_ ? LegTargets{} : targets;
}

bool ControlState::begin_storage(bool exclusive)
{
    const CriticalSection lock;
    return storage_interlock_.begin(feedback_.armed, feedback_.left_pwm, feedback_.right_pwm, exclusive);
}

void ControlState::end_storage()
{
    const CriticalSection lock;
    storage_interlock_.finish();
}

bool ControlState::storage_busy() const
{
    const CriticalSection lock;
    return storage_interlock_.busy();
}

bool ControlState::consume_storage_reset()
{
    const CriticalSection lock;
    return storage_interlock_.consumeReset();
}

bool ControlState::storage_blocks_control() const
{
    const CriticalSection lock;
    return storage_interlock_.blocksControl();
}

void ControlState::request_control_reset()
{
    const CriticalSection lock;
    storage_interlock_.requestReset();
}

void ControlState::set_installation_mode(bool enabled)
{
    const CriticalSection lock;
    if (installation_mode_ == enabled) { return; }
    installation_mode_ = enabled;
    installation_ready_ = false;
    parameters_.velocity_target = parameters_.difference_target = parameters_.roll_target = 0.0F;
    parameters_.leg_height = BalanceCompensation::minimum_leg_height_mm;
    parameters_.motion_command_received = false;
    feedback_.armed = false;
    feedback_.left_pwm = feedback_.right_pwm = 0;
    leg_targets_ = {};
    storage_interlock_.requestReset();
}

bool ControlState::installation_mode() const
{
    const CriticalSection lock;
    return installation_mode_;
}

void ControlState::set_installation_ready(bool ready)
{
    const CriticalSection lock;
    installation_ready_ = installation_mode_ && ready;
}

bool ControlState::installation_ready() const
{
    const CriticalSection lock;
    return installation_ready_;
}

} // namespace app
