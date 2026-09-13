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
}

LegTargets ControlState::leg_targets() const
{
    const CriticalSection lock;
    return leg_targets_;
}

void ControlState::publish_leg_targets(LegTargets targets)
{
    const CriticalSection lock;
    leg_targets_ = targets;
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

} // namespace app
