#pragma once

#include "ControlState.hpp"
#include "MotionParameterStorage.hpp"

namespace app {

// Owns saved-state diagnostics; ControlState remains the only live parameter owner.
class MotionPersistence final {
public:
    explicit MotionPersistence(ControlState& control) : control_(control) {}
    bool load(); // Bootstrap only, before command/motion/servo tasks start.
    MotionSettings::SaveResult save(bool recycle = false);
    bool has_saved_parameters() const { return has_saved_; }
    bool unsaved() const;
    static MotionSettings::Parameters snapshot(const ControlParameters& p);

private:
    ControlState& control_;
    MotionSettings::Parameters saved_{};
    bool has_saved_{};
};

} // namespace app
