/********************************************************************************
  * @file           : TaskReactor.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-19
  *******************************************************************************/

//#pragma once
#ifndef TASKREACTOR_HPP
#define TASKREACTOR_HPP

//c++ std library include
#include <functional>
//etl library include
#include "etl/string.h"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"
//user C++ library include
#include "BasicObject.hpp"
#include "TextCommandParser.hpp"

class TaskReactor : BasicObject {
    using SlotCallback = std::function<void()>;
public:
    TaskReactor()
        :reactorTask_(xTaskGetCurrentTaskHandle()) {}

    TaskReactor(const TaskReactor&) = delete;
    TaskReactor& operator=(const TaskReactor&) = delete;

    /**
     * @brief
     */
    using strCMD_t = text_command::ParsedCommand;

private:
    TaskHandle_t reactorTask_ = nullptr;
    std::array<SlotCallback, 32> SlotGroup{};
protected:

    /**
     * @brief
     * @return
     */
    uint32_t allocNotiftBit(){
        for(uint8_t i = 0; i < 32; ++i) {
            if(!SlotGroup[i]) { // 检查 std::function 是否为空
                return (1UL << i);
            }
        }
        return 0;
    }



public:
    /**
     * @brief connect the signal of LKObject.
     * @param sender
     * @param signal
     * @param slot
     * @example     TaskReactor task1;
     * /n           task1.connect();
     */
    template<typename Sender, typename SignalMethod, typename SlotLambda>
    bool connect(Sender* sender, SignalMethod signal, SlotLambda slot) {
        if (sender == nullptr) return false;
        uint32_t bitMask = allocNotiftBit();
        if (bitMask == 0) return false;                             // More than 32 signals failed.
        if (!sender->bindReactor(signal, reactorTask_, bitMask)) return false;
        // Put the slot function into the container
        int bitIndex = __builtin_ctz(bitMask);

        SlotGroup[bitIndex]=[sender, signal, slot]() {
            (sender->*signal)(slot);
        };
        return true;
    }

    /**
     * @brief
     */
    inline void taskLoop(TickType_t xTicksToWait =portMAX_DELAY, const std::function<void()>& func1=nullptr, const std::function<void()>& func2=nullptr) {
        uint32_t notifiedValue = 0;
        TickType_t xLastWakeTime = xTaskGetTickCount();
        while (true) {
            TickType_t xTimeNow = xTaskGetTickCount();
            TickType_t xTicksRemaining = portMAX_DELAY;
            if (func2 && xTicksToWait != portMAX_DELAY) {
                TickType_t elapsed = xTimeNow - xLastWakeTime;
                if (elapsed >= xTicksToWait) {
                    func2(); // 只有时间到了才执行
                    xLastWakeTime = xTaskGetTickCount();
                    elapsed = 0;
                }
                xTicksRemaining = xTicksToWait - elapsed;
            }
            if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, xTicksRemaining) == pdTRUE){
                for(uint8_t i = 0; notifiedValue > 0 && i < 32; i++) {
                    if(notifiedValue & (1UL << i)) {
                        if (SlotGroup[i]) {
                            SlotGroup[i]();
                        }
                        notifiedValue &= ~(1UL << i); // 清除已处理的位
                    }
                }
                if (func1)  func1();
            }
        }
    }

    static bool parseStrCMD(etl::string_view input, strCMD_t &strCmd){
        return text_command::parse(input, strCmd);
    }

    template<typename T>
    static bool parseStrArg(etl::string_view &str_arg,T& value){
        return text_command::parse_argument(str_arg, value);
    }
};



#endif //TASKREACTOR_HPP
