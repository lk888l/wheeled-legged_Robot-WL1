#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

class BasicObject
{
public:
    virtual ~BasicObject() = default;

    struct SignalContext
    {
        TaskHandle_t task_h = nullptr;
        std::uint32_t bitMask = 0;
    };

protected:
    static bool emit(const SignalContext& signal)
    {
        if (signal.task_h == nullptr || signal.bitMask == 0)
        {
            return false;
        }

        return xTaskNotify(signal.task_h, signal.bitMask, eSetBits) == pdPASS;
    }

    static bool emitFromISR(const SignalContext& signal,
                            BaseType_t* higher_priority_task_woken)
    {
        if (signal.task_h == nullptr || signal.bitMask == 0)
        {
            return false;
        }

        return xTaskNotifyFromISR(signal.task_h,
                                  signal.bitMask,
                                  eSetBits,
                                  higher_priority_task_woken) == pdPASS;
    }
};
