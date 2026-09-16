#pragma once

#include <algorithm>

namespace WheelPwm {

inline constexpr int maximum = 1000;
inline constexpr int compensation_ramp = 40;

// Continuous through zero: do not turn +/-1 count of noise into +/-deadzone.
// Above 40 requested counts the original minimum-drive compensation is retained.
constexpr int compensate(int request, unsigned deadzone) noexcept
{
    request = std::clamp(request, -maximum, maximum);
    const int magnitude = request < 0 ? -request : request;
    const int floor = static_cast<int>(std::min(deadzone, unsigned(maximum)));
    const int tapered = (floor * std::min(magnitude, compensation_ramp) +
                         compensation_ramp / 2) / compensation_ramp;
    const int output = std::max(magnitude, tapered);
    return request < 0 ? -output : output;
}

} // namespace WheelPwm
