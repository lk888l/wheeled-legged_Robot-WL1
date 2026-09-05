#pragma once

#include <array>
#include <charconv>
#include <cstdint>
#include "TextCommandParser.hpp"

namespace radio_frame {

// Wire format stays ASCII, four space-prefixed values with one decimal digit,
// zero-padded to 32 bytes. Invalid/unrepresentable frames are never transmitted.
[[nodiscard]] inline bool encode_telemetry(const std::array<float, 4U>& values,
                                           std::array<uint8_t, 32U>& payload)
{
    payload.fill(0U);
    char* cursor = reinterpret_cast<char*>(payload.data());
    char* const end = cursor + payload.size() - 1U; // Reserve the terminator.
    for (float value : values) {
        if (cursor == end || !text_command::finite_number(value)) {
            payload.fill(0U);
            return false;
        }
        *cursor++ = ' ';
        // Use integer formatting: floating to_chars pulls large conversion
        // tables into this Cortex-M image. Bound the conversion before casting.
        const bool negative = value < 0.0F;
        const float magnitude = negative ? -value : value;
        if (magnitude > 9999999.0F || end - cursor < (negative ? 4 : 3)) {
            payload.fill(0U);
            return false;
        }
        const uint32_t tenths = static_cast<uint32_t>(magnitude * 10.0F + 0.5F);
        if (negative) { *cursor++ = '-'; }
        const auto result = std::to_chars(cursor, end - 2, tenths / 10U);
        if (result.ec != std::errc{}) {
            payload.fill(0U);
            return false;
        }
        cursor = result.ptr;
        *cursor++ = '.';
        *cursor++ = static_cast<char>('0' + tenths % 10U);
    }
    *cursor = '\0';
    return true;
}

} // namespace radio_frame
