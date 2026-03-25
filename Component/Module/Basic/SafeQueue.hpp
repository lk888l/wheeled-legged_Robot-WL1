/********************************************************************************
  * @file           : SafeQueue.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : Applicable only to single-core mcu.
  * @version        : v1.0
  * @date           : 26-3-21
  *******************************************************************************/

#ifdef __GNUC__
#pragma once
#endif //__GNUC__
#ifndef __SAFEQUEUE_HPP
#define __SAFEQUEUE_HPP

//ETL library include
#include "etl/queue.h"
#include "etl/string.h"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"

template <typename T, size_t MAX_SIZE>
class SafeQueue {
private:
    etl::queue<T, MAX_SIZE> queue;

public:
    /**
     * @brief 中断安全入队
     * @param item
     * @return
     */
    bool push_FromISR(const T& item) {
        // STM32 单核下，简单关闭中断即可保证绝对安全
        uint32_t mask = taskENTER_CRITICAL_FROM_ISR();
        if (queue.full()) {
            taskEXIT_CRITICAL_FROM_ISR(mask);
            return false;
        }
        queue.push(item);
        taskEXIT_CRITICAL_FROM_ISR(mask);
        return true;
    }

    /**
     * @brief this is safe push in task
     * @param item
     * @return
     */
    bool push(T& item){
        taskENTER_CRITICAL();
        if (queue.full()) {
            taskEXIT_CRITICAL();
            return false;
        }
        queue.push(item);
        taskEXIT_CRITICAL();
        return true;
    }

    /**
     * @brief normal push(isn't safe push)
     * @param item
     * @return
     */
    bool push_normal(const T& item){
        if (queue.full()) {
            return false;
        }
        queue.push(item);
        return true;
    }

    /**
     * @brief 任务中安全出队
     * @param item
     * @return
     */
    bool pop(T& item) {
        taskENTER_CRITICAL();
        if (queue.empty()) {
            taskEXIT_CRITICAL();
            return false;
        }
        item = queue.front();
        queue.pop();
        taskEXIT_CRITICAL();
        return true;
    }

    /**
 * @brief 中断安全出队
 * @param item
 * @return
 */
    bool pop_FromISR(T& item) {
        // STM32 单核下，简单关闭中断即可保证绝对安全
        uint32_t mask = taskENTER_CRITICAL_FROM_ISR();
        if (queue.empty()) {
            taskEXIT_CRITICAL_FROM_ISR(mask);
            return false;
        }
        item = queue.front();
        queue.pop();
        taskEXIT_CRITICAL_FROM_ISR(mask);
        return true;
    }

    /**
     * @brief normal pop(isn't safe push)
     * @param item
     * @return
     */
    bool pop_normal(T& item){
        if (queue.empty()) {
            return false;
        }
        item = queue.front();
        queue.pop();
        return true;
    }

    bool empty(){
        taskENTER_CRITICAL();
        bool isempty = queue.empty();
        taskEXIT_CRITICAL();
        return isempty;
    }

    bool empty_normal(){
        return queue.empty();
    }
};


#endif //__SAFEQUEUE_HPP
