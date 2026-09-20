#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "BalanceCompensation.hpp"

namespace app {

enum class ColdStartPhase : std::uint8_t {
    waiting, raising, settling, ramping, driving, handing_over, complete, aborted
};
enum class ColdStartAbort : std::uint8_t { none, interrupted, attitude, imu, timeout, timing, encoder };

inline const char* cold_start_phase_name(ColdStartPhase phase) noexcept
{
    switch (phase) {
    case ColdStartPhase::waiting: return "waiting";
    case ColdStartPhase::raising: return "raising";
    case ColdStartPhase::settling: return "settling";
    case ColdStartPhase::ramping: return "ramping";
    case ColdStartPhase::driving: return "driving";
    case ColdStartPhase::handing_over: return "handover";
    case ColdStartPhase::complete: return "complete";
    case ColdStartPhase::aborted: return "aborted";
    }
    return "unknown";
}

// Stand launch is opt-in; ordinary application startup leaves it inactive.
// All time inputs are elapsed milliseconds; no RTOS or
// hardware dependencies. The stand provides support while both legs extend.
class ColdStartLaunch {
public:
    static constexpr float midpoint_mm = (BalanceCompensation::minimum_leg_height_mm +
        BalanceCompensation::maximum_leg_height_mm) / 2.0F;
    static constexpr float launch_height_mm = midpoint_mm + 8.0F;
    static_assert(launch_height_mm <= BalanceCompensation::maximum_leg_height_mm);
    static constexpr float wheel_diameter_mm = 44.0F;
    static constexpr float travel_mm = 120.0F;
    // Match tele_firmware and the miniapp: physical forward is negative encoder RPM.
    static constexpr float forward_rpm = -30.0F;
    static constexpr std::uint32_t stable_ms = 500U;
    static constexpr std::uint32_t raise_ms = 3000U;
    static constexpr std::uint32_t settle_ms = 500U;
    static constexpr std::uint32_t ramp_ms = 1000U;
    static constexpr std::uint32_t handover_ms = 1500U;
    static constexpr std::uint32_t speed_handover_ms = 300U;
    static constexpr std::uint32_t drive_timeout_ms = 6000U; // Includes the PWM ramp.

    explicit ColdStartLaunch(bool enabled = false) noexcept
        : phase_(enabled ? ColdStartPhase::waiting : ColdStartPhase::complete) {}

    [[nodiscard]] ColdStartPhase phase() const noexcept { return phase_; }
    [[nodiscard]] ColdStartAbort abort_reason() const noexcept { return abort_; }
    [[nodiscard]] float distance_mm() const noexcept { return distance_mm_; }
    [[nodiscard]] bool owns_control() const noexcept
    { return phase_ != ColdStartPhase::complete && phase_ != ColdStartPhase::aborted; }
    [[nodiscard]] bool balancing() const noexcept
    { return phase_ == ColdStartPhase::ramping || phase_ == ColdStartPhase::driving ||
             phase_ == ColdStartPhase::handing_over; }

    void cancel(ColdStartAbort reason = ColdStartAbort::interrupted) noexcept
    {
        if (owns_control()) { phase_ = ColdStartPhase::aborted; abort_ = reason; }
    }

    void update(std::uint32_t dt_ms, float pitch, float roll, float gyro_rate,
                float speed_request, float turn_request) noexcept
    {
        if (!owns_control()) { return; }
        const bool finite = std::isfinite(pitch) && std::isfinite(roll) &&
            std::isfinite(gyro_rate) && std::isfinite(speed_request) && std::isfinite(turn_request);
        if (phase_ == ColdStartPhase::waiting) {
            // A supported body need not be at the free-balancing pitch bias.
            // Keep the 500 ms rest requirement, using raw body attitude here.
            if (!finite || dt_ms != 10U || std::fabs(pitch) > 20.0F ||
                std::fabs(roll) > 10.0F || std::fabs(gyro_rate) > 20.0F ||
                std::fabs(speed_request) >= 1.0F || std::fabs(turn_request) >= 1.0F) {
                elapsed_ms_ = 0U;
                return;
            }
            elapsed_ms_ += dt_ms;
            if (elapsed_ms_ >= stable_ms) { enter(ColdStartPhase::raising); }
            return;
        }
        if (!finite || std::fabs(pitch) > 30.0F || std::fabs(roll) > 30.0F) {
            cancel(ColdStartAbort::attitude);
            return;
        }
        if (dt_ms > 50U) { cancel(ColdStartAbort::timing); return; }
        elapsed_ms_ += dt_ms;
        switch (phase_) {
        case ColdStartPhase::raising:
            if (elapsed_ms_ >= raise_ms) { enter(ColdStartPhase::settling); }
            break;
        case ColdStartPhase::settling:
            if (elapsed_ms_ >= settle_ms) { enter(ColdStartPhase::ramping); }
            break;
        case ColdStartPhase::ramping:
        case ColdStartPhase::driving:
            drive_ms_ += dt_ms;
            if (phase_ == ColdStartPhase::ramping && elapsed_ms_ >= ramp_ms) {
                enter(ColdStartPhase::driving);
            }
            if (phase_ == ColdStartPhase::driving && distance_mm_ >= travel_mm) {
                enter(ColdStartPhase::handing_over);
            } else if (drive_ms_ >= drive_timeout_ms) {
                cancel(ColdStartAbort::timeout);
            }
            break;
        case ColdStartPhase::handing_over:
            if (elapsed_ms_ >= handover_ms) { enter(ColdStartPhase::complete); }
            break;
        default: break;
        }
    }

    // HallEncoder::getRPM uses a configured 50 ms denominator. Multiplying by
    // 50/60000 recovers the actual signed encoder turns, even after a late read.
    // Call exactly once per pair of encoder reads; pre-ramp movement is discarded.
    void observe_wheel_rpm(float left, float right) noexcept
    {
        if (phase_ != ColdStartPhase::ramping && phase_ != ColdStartPhase::driving) { return; }
        if (!std::isfinite(left) || !std::isfinite(right)) { cancel(ColdStartAbort::encoder); return; }
        distance_mm_ -= (left + right) * 0.5F / 1200.0F *
            (3.14159265358979323846F * wheel_diameter_mm);
    }

    [[nodiscard]] float normal_weight() const noexcept
    {
        return !owns_control() ? 1.0F : phase_ == ColdStartPhase::handing_over
            ? smooth(elapsed_ms_, handover_ms) : 0.0F;
    }
    [[nodiscard]] float output_gain() const noexcept
    {
        if (!owns_control()) { return phase_ == ColdStartPhase::complete ? 1.0F : 0.0F; }
        if (!balancing()) { return 0.0F; }
        return phase_ == ColdStartPhase::ramping ? smooth(elapsed_ms_, ramp_ms) : 1.0F;
    }
    [[nodiscard]] float height(float normal_height) const noexcept
    {
        if (!owns_control()) { return normal_height; }
        if (phase_ == ColdStartPhase::waiting) { return BalanceCompensation::minimum_leg_height_mm; }
        if (phase_ == ColdStartPhase::raising) {
            return blend(BalanceCompensation::minimum_leg_height_mm, launch_height_mm,
                         smooth(elapsed_ms_, raise_ms));
        }
        return blend(launch_height_mm, normal_height, normal_weight());
    }
    [[nodiscard]] float speed(float normal_speed) const noexcept
    {
        if (!owns_control()) { return normal_speed; }
        if (!balancing()) { return 0.0F; }
        return phase_ == ColdStartPhase::handing_over
            ? blend(forward_rpm, normal_speed, smooth(elapsed_ms_, speed_handover_ms))
            : forward_rpm * output_gain();
    }

private:
    static float blend(float from, float to, float weight) noexcept { return from + (to - from) * weight; }
    static float smooth(std::uint32_t elapsed, std::uint32_t duration) noexcept
    {
        const float x = std::min(1.0F, static_cast<float>(elapsed) / duration);
        return x * x * (3.0F - 2.0F * x);
    }
    void enter(ColdStartPhase phase) noexcept { phase_ = phase; elapsed_ms_ = 0U; }
    ColdStartPhase phase_;
    ColdStartAbort abort_{ColdStartAbort::none};
    std::uint32_t elapsed_ms_{}, drive_ms_{};
    float distance_mm_{};
};
} // namespace app
