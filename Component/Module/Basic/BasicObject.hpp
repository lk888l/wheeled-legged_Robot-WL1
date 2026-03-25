/********************************************************************************
  * @file           : BasicObject.hpp
  * @author         : Luka
  * @brief          : Embedded object base class based on freeRTOS.
  * @attention      : Only using the environment of FreeRTOS.
  * @date           : 26-3-22
  * @version        : V1.0
  *******************************************************************************/


#ifdef __GNUC__
#pragma once
#endif //__GNUC__
#ifndef __BASICOJECT_HPP
#define __BASICOJECT_HPP

//c++ std library include
#include <functional>
#include <array>
//freeRtos library include
#include "FreeRTOS.h"
#include "task.h"


class BasicObject {
public:
    virtual ~BasicObject() = default;
    struct SignalContext {
        TaskHandle_t task_h = nullptr;
        uint32_t bitMask = 0;
    };

//    /**
//     * @brief
//     * @param signal
//     */
//    template<typename SignalPtr>
//    bool bindReactor(SignalPtr signalFunc, SignalContext signal){
//        return false;
//    }

    /**
     * @brief
     * @param signal
     * @param pxHigherPriorityTaskWoken
     */
    void emitFromISR(const SignalContext &signal, BaseType_t* pxHigherPriorityTaskWoken) {
        if (signal.task_h != nullptr && signal.bitMask != 0) {
            xTaskNotifyFromISR(signal.task_h, signal.bitMask, eSetBits, pxHigherPriorityTaskWoken);
        }
    }
    /**
     * @brief
     * @param signal
     */
    void emit(SignalContext signal){
        if (signal.task_h != nullptr && signal.bitMask != 0){
            xTaskNotify(signal.task_h, signal.bitMask, eSetBits);
        }
    }
};


#endif //__BASICOJECT_HPP
