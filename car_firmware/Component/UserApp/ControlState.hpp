#pragma once

#include <array>

namespace app {

struct PidGains { float kp; float ki; float kd; };

struct ControlParameters {
    PidGains angle{70.0F, 0.0F, 60.0F};
    PidGains velocity{0.05F, 0.008F, 0.0F};
    PidGains difference{2.0F, 0.001F, 0.0F};
    PidGains roll{0.0F, -0.4F, 0.0F};
    float angle_bias{12.6F};
    float velocity_target{0.0F};
    float difference_target{0.0F};
    float leg_height{44.5F};
    float roll_target{0.0F};
    bool show_imu{false};
    bool show_rpm{false};
};

struct LegTargets { float left{0.0F}; float right{0.0F}; };

struct ControlFeedback {
    std::array<float, 3U> euler{};
    float angle_kp{70.0F};
    float angle_bias{12.6F};
};

// Value types above are portable; the snapshot implementation is single-core
// FreeRTOS. Only commands write parameters; only motion writes feedback/legs.
// Callers receive copies, never mutable references to shared state.
class ControlState final {
public:
    [[nodiscard]] ControlParameters parameters() const;
    void set_parameters(const ControlParameters& parameters);
    [[nodiscard]] ControlFeedback feedback() const;
    void publish_feedback(const ControlFeedback& feedback);
    [[nodiscard]] LegTargets leg_targets() const;
    void publish_leg_targets(LegTargets targets);

private:
    ControlParameters parameters_{};
    ControlFeedback feedback_{};
    LegTargets leg_targets_{};
};

} // namespace app
