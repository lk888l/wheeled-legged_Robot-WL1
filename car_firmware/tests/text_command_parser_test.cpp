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

    std::cout << "text command parser tests passed\n";
    return 0;
}
