#pragma once

#include "etl/string_view.h"

namespace text_command {

struct ParsedCommand {
    etl::string_view command{};
    etl::string_view args{};
};

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
