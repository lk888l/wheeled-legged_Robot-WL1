#pragma once

#include "FreeRTOS.h"
#include "task.h"

namespace app {

// Single-core FreeRTOS task context only. Use solely for short value copies;
// never perform I/O, calculation, logging, or invoke callbacks while held.
class CriticalSection final {
public:
    CriticalSection() { taskENTER_CRITICAL(); }
    ~CriticalSection() { taskEXIT_CRITICAL(); }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
};

} // namespace app
