#pragma once

#include <array>
#include <cstdint>
#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "MotionParameters.hpp"
#include "MotionStorageInterlock.hpp"

namespace app {

using PidGains = MotionSettings::PidGains;

struct ControlParameters {
    PidGains angle{MotionSettings::Parameters{}.angle};
    PidGains velocity{0.05F, 0.008F, 0.0F};
    PidGains difference{2.0F, 0.001F, 0.0F};
    PidGains roll{0.0F, -0.4F, 0.0F};
    // Runtime calibration at minimum leg height; the height correction is separate.
    float angle_bias{BalanceCompensation::default_minimum_bias_degrees};
    float velocity_target{0.0F};
    float difference_target{0.0F};
    float leg_height{44.5F};
    float roll_target{0.0F};
    bool show_imu{false};
    bool show_rpm{false};
    // -p tunes the 61.5 mm reference gain; -manual explicitly selects fixed Kp.
    bool angle_kp_auto{true};
    // Only accepted movement commands renew this deadline; PID tuning does not.
    uint32_t motion_command_tick{};
    bool motion_command_received{};
};

struct LegTargets { float left{44.5F}; float right{44.5F}; };

struct ControlFeedback {
    std::array<float, 3U> euler{};
    float angle_kp{70.0F};
    float angle_bias{BalanceCompensation::default_minimum_bias_degrees};
    float pitch_error{};
    int left_pwm{};
    int right_pwm{};
    bool armed{};
    bool imu_valid{};
    uint32_t loop_count{};
    uint32_t sample_tick{};
    uint32_t max_sample_gap_ticks{};
    uint32_t deadline_misses{};
    bool remote_timed_out{};
    float velocity_target{};
    float difference_target{};
    float roll_target{};
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
    [[nodiscard]] bool begin_storage(bool exclusive);
    void end_storage();
    [[nodiscard]] bool storage_busy() const;
    [[nodiscard]] bool storage_blocks_control() const;
    [[nodiscard]] bool consume_storage_reset();
    void request_control_reset();

private:
    ControlParameters parameters_{};
    ControlFeedback feedback_{};
    LegTargets leg_targets_{};
    MotionStorageInterlock storage_interlock_{};
};

} // namespace app
