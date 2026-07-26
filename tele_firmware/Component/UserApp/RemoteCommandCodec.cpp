#include "RemoteCommandCodec.hpp"

#include <algorithm>
#include <cstdio>

bool RemoteCommandCodec::encode(
    const RemoteControlSnapshot& command, Payload& payload) noexcept
{
    std::array<char, payload_size> text{};
    const int length = std::snprintf(
        text.data(),
        text.size(),
        "R %.1f %.1f %.1f %.1f",
        static_cast<double>(command.turn),
        static_cast<double>(-command.speed),
        static_cast<double>(command.roll_degrees),
        static_cast<double>(command.leg_height_mm));

    if (length <= 0 || static_cast<std::size_t>(length) >= text.size()) {
        payload.fill(0U);
        return false;
    }

    payload.fill(0U);
    std::copy_n(
        reinterpret_cast<const std::uint8_t*>(text.data()),
        static_cast<std::size_t>(length),
        payload.begin());
    return true;
}
