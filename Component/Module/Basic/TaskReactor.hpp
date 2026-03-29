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
#include "etl/vector.h"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"
//user C++ library include
#include "BasicObject.hpp"
//stm-Hal library include
#include "main.h"

class TaskReactor : BasicObject {
    using SlotCallback = std::function<void()>;
public:
    TaskReactor()
        :reactorTask_(xTaskGetCurrentTaskHandle()){
        SlotGroup.uninitialized_resize(1);
    }
private:
    std::array<SlotCallback, 32> slots; // 32个Bit对应32个槽
    TaskHandle_t reactorTask_ = nullptr;
    etl::vector<std::function<void()>, 32> SlotGroup{nullptr};
protected:

    /**
     * @brief
     * @return
     */
    uint32_t allocNotiftBit(){
        uint8_t i{};
        for(const auto& slot:SlotGroup){
            if(!slot){
                return (1 << i);
            }
            i++;
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
        uint32_t bitMask = allocNotiftBit();
        if (bitMask == 0) return false;                             // More than 32 signals failed.
        sender->bindReactor(signal, reactorTask_, bitMask);      // execute LKObject bindReactorBit.
        // Put the slot function into the container
        int bitIndex = __builtin_ctz(bitMask);
        if(bitIndex > SlotGroup.size())     {SlotGroup.uninitialized_resize(bitIndex);}
        SlotGroup[bitIndex]=[sender, signal, slot]() {
            (sender->*signal)(slot);
        };
        return true;
    }

    /**
     * @brief
     */
    inline void taskLoop(TickType_t xTicksToWait =portMAX_DELAY, const std::function<void()>& func=nullptr) {
        uint32_t notifiedValue = 0;
        while (true) {
            if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, xTicksToWait) == pdTRUE){
                for(uint8_t i=0;i<SlotGroup.size();i++){
                    if(notifiedValue & (1 << i)){
                        if (SlotGroup[i]) {
                            SlotGroup[i](); // 执行槽函数
                        }
                    }
                }
            }
            func();
        }
    }
};


#endif //TASKREACTOR_HPP
