#pragma once
#include <cassert>
#include <cstdint>

using BaseType_t = int;
using UBaseType_t = uint32_t;
using TickType_t = uint32_t;
using StackType_t = uint32_t;
using configSTACK_DEPTH_TYPE = uint32_t;
using TaskHandle_t = void*;
using TaskFunction_t = void (*)(void*);
struct StaticTask_t { uint32_t words[32]{}; };

inline constexpr BaseType_t pdTRUE = 1;
inline constexpr BaseType_t pdFALSE = 0;
inline constexpr BaseType_t pdPASS = 1;
inline constexpr UBaseType_t configMAX_PRIORITIES = 56U;
inline constexpr UBaseType_t configTIMER_TASK_PRIORITY = 2U;
inline constexpr UBaseType_t tskIDLE_PRIORITY = 0U;
inline constexpr TickType_t portMAX_DELAY = UINT32_MAX;
#define pdMS_TO_TICKS(ms) static_cast<TickType_t>(ms)
#define configASSERT(condition) assert(condition)

namespace fake_rtos {
inline int critical_depth = 0;
inline unsigned critical_entries = 0U;
}
#define taskENTER_CRITICAL() do { ++fake_rtos::critical_depth; ++fake_rtos::critical_entries; } while (false)
#define taskEXIT_CRITICAL() do { assert(fake_rtos::critical_depth > 0); --fake_rtos::critical_depth; } while (false)
