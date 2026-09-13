#pragma once

#include <charconv>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include "etl/string_view.h"

namespace text_command {

struct ParsedCommand {
    etl::string_view command{};
    etl::string_view args{};
};

// Bit classification keeps validation valid even in legacy -Ofast builds,
// whose finite-math assumptions can optimize std::isfinite() away.
template<typename T>
[[nodiscard]] bool finite_number(T value)
{
    if constexpr (std::is_floating_point_v<T>) {
        static_assert(std::numeric_limits<T>::is_iec559);
        if constexpr (sizeof(T) == sizeof(uint32_t)) {
            uint32_t bits{};
            std::memcpy(&bits, &value, sizeof(bits));
            return (bits & 0x7F800000U) != 0x7F800000U;
        } else {
            static_assert(sizeof(T) == sizeof(uint64_t));
            uint64_t bits{};
            std::memcpy(&bits, &value, sizeof(bits));
            return (bits & UINT64_C(0x7FF0000000000000)) != UINT64_C(0x7FF0000000000000);
        }
    }
    return true;
}

// Consume one complete numeric token. Whitespace is accepted; suffixes, NaN,
// infinity and overflow are rejected. Failure leaves both arguments intact.
template<typename T>
[[nodiscard]] bool parse_argument(etl::string_view& args, T& value)
{
    const size_t start = args.find_first_not_of(" \t\r\n");
    if (start == etl::string_view::npos) { return false; }
    auto remaining = args.substr(start);
    const size_t separator = remaining.find_first_of(" \t\r\n");
    const auto token = remaining.substr(0U, separator);
    T candidate{};
    const auto result = std::from_chars(token.data(), token.data() + token.size(), candidate);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size() ||
        !finite_number(candidate)) {
        return false;
    }
    remaining = separator == etl::string_view::npos
        ? etl::string_view{} : remaining.substr(separator);
    const size_t next = remaining.find_first_not_of(" \t\r\n");
    args = next == etl::string_view::npos ? etl::string_view{} : remaining.substr(next);
    value = candidate;
    return true;
}

/**
 * Split an ASCII command frame into command and argument views.
 *
 * UART receive-to-idle commonly includes CR/LF in the received span, so both
 * ends are trimmed before parsing. Spaces and tabs separate the command from
 * its optional arguments. The returned views continue to reference input.
 */
[[nodiscard]] inline bool parse(etl::string_view input, ParsedCommand& parsed)
{
    constexpr const char* kFrameWhitespace = " \t\r\n";
    constexpr const char* kArgumentSeparators = " \t";

    const size_t start = input.find_first_not_of(kFrameWhitespace);
    if (start == etl::string_view::npos) {
        parsed = {};
        return false;
    }

    const size_t end = input.find_last_not_of(kFrameWhitespace);
    input = input.substr(start, end - start + 1U);

    const size_t separator = input.find_first_of(kArgumentSeparators);
    if (separator == etl::string_view::npos) {
        parsed.command = input;
        parsed.args = {};
        return true;
    }

    parsed.command = input.substr(0U, separator);
    etl::string_view args = input.substr(separator + 1U);
    const size_t argument_start = args.find_first_not_of(kArgumentSeparators);
    parsed.args = argument_start == etl::string_view::npos
                      ? etl::string_view{}
                      : args.substr(argument_start);
    return !parsed.command.empty();
}

} // namespace text_command
