#include <cassert>
#include <iostream>

#include "TextCommandParser.hpp"

namespace {

void expect_command(etl::string_view frame,
                    etl::string_view expected_command,
                    etl::string_view expected_args)
{
    text_command::ParsedCommand parsed{};
    assert(text_command::parse(frame, parsed));
    assert(parsed.command == expected_command);
    assert(parsed.args == expected_args);
}

} // namespace

int main()
{
    expect_command("ping\n", "ping", "");
    expect_command("  status\r\n", "status", "");
    expect_command("ping   \r\n", "ping", "");
    expect_command("motor 100 100\r\n", "motor", "100 100");
    expect_command("anglepid\t-p 70\r\n", "anglepid", "-p 70");

    text_command::ParsedCommand parsed{};
    assert(!text_command::parse(" \t\r\n", parsed));
    assert(parsed.command.empty());
    assert(parsed.args.empty());

    etl::string_view numbers = " \t -2.5\t 12  \r\n";
    float decimal{};
    unsigned integer{};
    assert(text_command::parse_argument(numbers, decimal) && decimal == -2.5F);
    assert(text_command::parse_argument(numbers, integer) && integer == 12U);
    assert(numbers.empty());
    for (etl::string_view invalid : {"", "  ", "12junk", "nan", "-nan", "inf", "-inf", "1e1000"}) {
        const auto before = invalid;
        decimal = 7.0F;
        assert(!text_command::parse_argument(invalid, decimal));
        assert(decimal == 7.0F && invalid == before);
    }
    etl::string_view unsigned_negative = "-1";
    assert(!text_command::parse_argument(unsigned_negative, integer));
    etl::string_view overflow = "256";
    uint8_t small{};
    assert(!text_command::parse_argument(overflow, small));

    std::cout << "text command parser tests passed\n";
    return 0;
}
