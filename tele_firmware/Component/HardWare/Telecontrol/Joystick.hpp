#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>

class Joystick final {
public:
    Joystick(
        std::uint16_t adc_min,
        std::uint16_t adc_max,
        std::uint16_t adc_center,
        std::uint16_t deadzone,
        float output_min,
        float output_max) noexcept
        : adc_min_(adc_min),
          adc_max_(adc_max),
          adc_center_(adc_center),
          deadzone_(deadzone),
          output_min_(output_min),
          output_max_(output_max)
    {
    }

    [[nodiscard]] float convert(std::uint16_t raw_adc) const noexcept
    {
        raw_adc = std::clamp(raw_adc, adc_min_, adc_max_);

        const std::int32_t offset =
            static_cast<std::int32_t>(raw_adc) - static_cast<std::int32_t>(adc_center_);
        const float output_center = (output_min_ + output_max_) * 0.5F;
        if (std::abs(offset) <= static_cast<std::int32_t>(deadzone_)) {
            return output_center;
        }

        if (offset > 0) {
            const std::uint32_t input_start =
                static_cast<std::uint32_t>(adc_center_) + deadzone_;
            if (input_start >= adc_max_) {
                return output_max_;
            }
            const float ratio =
                static_cast<float>(raw_adc - input_start) /
                static_cast<float>(adc_max_ - input_start);
            return output_center + ratio * (output_max_ - output_center);
        }

        const std::int32_t input_end =
            static_cast<std::int32_t>(adc_center_) - static_cast<std::int32_t>(deadzone_);
        if (input_end <= static_cast<std::int32_t>(adc_min_)) {
            return output_min_;
        }
        const float ratio =
            static_cast<float>(raw_adc - adc_min_) /
            static_cast<float>(input_end - static_cast<std::int32_t>(adc_min_));
        return output_min_ + ratio * (output_center - output_min_);
    }

private:
    const std::uint16_t adc_min_;
    const std::uint16_t adc_max_;
    const std::uint16_t adc_center_;
    const std::uint16_t deadzone_;
    const float output_min_;
    const float output_max_;
};
