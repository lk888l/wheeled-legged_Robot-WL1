#pragma once
#include <cstdint>
using TickType_t = std::uint32_t;
#define pdMS_TO_TICKS(value) (value)
#define taskENTER_CRITICAL() ((void)0)
inline void (*test_critical_exit_hook)() = nullptr;
inline int test_hook_countdown = 0;
inline void test_critical_exit()
{
    if (test_critical_exit_hook != nullptr && --test_hook_countdown == 0) {
        const auto hook = test_critical_exit_hook;
        test_critical_exit_hook = nullptr;
        hook();
    }
}
#define taskEXIT_CRITICAL() test_critical_exit()
