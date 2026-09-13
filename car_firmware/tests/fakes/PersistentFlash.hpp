#pragma once
#include <array>
#include <functional>
#include "MotionParameterJournal.hpp"
#include "../test_check.hpp"
#include "task.h"

namespace fake_flash {
struct Flash {
    static constexpr std::size_t capacity_bytes = 84U * 8U;
    std::array<std::uint32_t, capacity_bytes / 4> words;
    unsigned writes{}, erases{};
    bool fail_write{};
    std::function<void()> on_write;
    Flash() { words.fill(0xFFFFFFFFU); }
    std::uint32_t readWord(std::size_t offset) { return words.at(offset / 4); }
    bool programWord(std::size_t offset, std::uint32_t value)
    {
        CHECK(fake_rtos::critical_depth == 0);
        if (on_write) on_write();
        if (fail_write) return false;
        CHECK(offset % 4 == 0 && words.at(offset / 4) == 0xFFFFFFFFU);
        words[offset / 4] = value;
        ++writes;
        return true;
    }
    bool erase()
    {
        CHECK(fake_rtos::critical_depth == 0);
        words.fill(0xFFFFFFFFU);
        ++erases;
        return true;
    }
};
inline Flash device;
inline void reset() { device = Flash{}; }
} // namespace fake_flash
