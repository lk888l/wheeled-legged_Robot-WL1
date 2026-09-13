#include "MotionPersistence.hpp"

namespace app {

MotionSettings::Parameters MotionPersistence::snapshot(const ControlParameters& p)
{
    return {p.angle_bias, p.angle, p.velocity, p.difference, p.roll,
        BalanceCompensation::clampLegHeight(p.leg_height), p.roll_target, p.angle_kp_auto};
}

bool MotionPersistence::load()
{
    has_saved_ = MotionSettings::loadFromFlash(saved_);
    if (!has_saved_) return false;
    // Explicit whitelist keeps movement deadlines, drive targets and diagnostics transient.
    ControlParameters p;
    p.angle_bias = saved_.minimum_pitch_bias;
    p.angle = saved_.angle;
    p.velocity = saved_.velocity;
    p.difference = saved_.difference;
    p.roll = saved_.roll;
    p.leg_height = saved_.leg_height;
    p.roll_target = saved_.roll_target;
    p.angle_kp_auto = saved_.angle_kp_auto;
    control_.set_parameters(p);
    control_.publish_leg_targets({p.leg_height, p.leg_height});
    ControlFeedback feedback;
    feedback.angle_bias = BalanceCompensation::pitchBias(p.angle_bias, p.leg_height);
    feedback.angle_kp = p.angle_kp_auto
        ? MotionSettings::effectiveAngleKp(p.angle.kp, p.leg_height) : p.angle.kp;
    control_.publish_feedback(feedback);
    return true;
}

MotionSettings::SaveResult MotionPersistence::save(bool recycle)
{
    if (!control_.begin_storage()) return MotionSettings::SaveResult::busy;
    const auto result = MotionSettings::saveToFlash(snapshot(control_.parameters()), recycle);
    has_saved_ = MotionSettings::loadFromFlash(saved_);
    control_.end_storage();
    return result;
}

bool MotionPersistence::unsaved() const
{
    return !has_saved_ || !MotionSettings::sameParameters(snapshot(control_.parameters()), saved_);
}

} // namespace app
