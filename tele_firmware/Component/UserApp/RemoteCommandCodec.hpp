#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "RemoteControlState.hpp"

class RemoteCommandCodec final {
public:
    static constexpr std::size_t payload_size = 32U;
    using Payload = std::array<std::uint8_t, payload_size>;

    [[nodiscard]] static bool encode(
        const RemoteControlSnapshot& command, Payload& payload) noexcept;
};
