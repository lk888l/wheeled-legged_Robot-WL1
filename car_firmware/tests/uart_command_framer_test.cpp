#include <string>
#include <vector>
#include "UartCommandFramer.hpp"
#include "test_check.hpp"

int main()
{
    // Every possible split, including within a valid numeric prefix, must emit
    // exactly one complete command only after its terminator has arrived.
    for (const std::string wire : {"@R -100 -100 -18 78.5\n", "@anglepid -p 75.25\r\n"}) {
        for (size_t split = 0; split <= wire.size(); ++split) {
            app::UartCommandFramer framer;
            std::vector<std::string> frames;
            const auto emit = [&](etl::string_view v) { frames.emplace_back(v.data(), v.size()); };
            framer.feed({wire.data(), split}, emit);
            if (split < wire.find_first_of("\r\n") + 1U) { CHECK(frames.empty()); }
            framer.feed({wire.data() + split, wire.size() - split}, emit);
            CHECK(frames.size() == 1U);
            CHECK(frames[0] == wire.substr(1U, wire.find_first_of("\r\n") - 1U));
        }
    }
    app::UartCommandFramer framer;
    std::vector<std::string> frames;
    const auto emit = [&](etl::string_view v) { frames.emplace_back(v.data(), v.size()); };
    framer.feed("R 0 0 0 44.5", emit);
    framer.feed("anglepid -p 80\r\nanglepid\nping\n", emit);
    CHECK((frames == std::vector<std::string>{"R 0 0 0 44.5", "anglepid -p 80", "anglepid", "ping"}));
    frames.clear();
    const std::string batch = "@R 0 0 0 61.5\n@anglebias 10.5\n@rollpid -d 1\n";
    for (char byte : batch) { framer.feed({&byte, 1U}, emit); }
    CHECK((frames == std::vector<std::string>{"R 0 0 0 61.5", "anglebias 10.5", "rollpid -d 1"}));
    frames.clear();
    framer.feed("@anglepid -p 7", emit);
    framer.discard_partial();
    framer.feed("5\n@ping\n", emit);
    CHECK((frames == std::vector<std::string>{"ping"}));
    frames.clear();
    const std::string oversized = "@" + std::string(33U, 'x') + "\n@ping\n";
    framer.feed(oversized.c_str(), emit);
    CHECK((frames == std::vector<std::string>{"ping"}));
    frames.clear();
    framer.feed("@invalid partial@R 0 0 0 44.5\n", emit);
    CHECK((frames == std::vector<std::string>{"R 0 0 0 44.5"}));
    frames.clear();
    framer.feed("@R 1 2\x01 3 60\n", emit);
    CHECK(frames.empty());
    framer.discard_partial();
    framer.feed("R 1 2 3 60", emit);
    CHECK(frames.empty());
    framer.feed("R 0 0 0 44.5", emit);
    CHECK((frames == std::vector<std::string>{"R 0 0 0 44.5"}));
    frames.clear();
    framer.feed("@anglepid -p 7", emit, 10U);
    framer.expire(310U, 300U);
    framer.feed("5\n", emit, 310U);
    CHECK(frames.empty()); // A delayed tail cannot start stale motion/tuning.
    framer.feed("@ping\n", emit, 311U);
    CHECK((frames == std::vector<std::string>{"ping"}));
    frames.clear();
    framer.feed("@R 0 0 0 6", emit, UINT32_MAX - 100U);
    framer.expire(150U, 300U);
    framer.feed("1.5\n", emit, 150U);
    CHECK((frames == std::vector<std::string>{"R 0 0 0 61.5"}));
    frames.clear();
    framer.feed("@R 0 0 0 6", emit, UINT32_MAX - 100U);
    framer.expire(250U, 300U);
    framer.feed("1.5\n", emit, 250U);
    CHECK(frames.empty());
}
