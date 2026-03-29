/********************************************************************************
  * @file           : LkUart.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-4
  * @version        : v1.0
  *******************************************************************************/

//#pragma once
#ifndef __LKUART_HPP
#define __LKUART_HPP

//c++ standard library include
#include "etl/format.h"
// stm-Hal library include
#include "main.h"
// freeRTOS library include
// LK Library include
#include "SafeQueue.hpp"
#include "BasicObject.hpp"


template<size_t TX_BufferSize = 128, size_t TX_BufDepth = 10, size_t RX_BufferSize = 128, size_t RX_BufDepth = 10>
class LkUart : BasicObject{
public:
    //receive Message type
    explicit LkUart(UART_HandleTypeDef* huart)
            : HUart(huart), dmaBusy_(false), curBufIndex_Tx(0),curBufIndex_Rx(0)
    {
        instance_ = this;
        //Put the indices of all buffers into the free queue
        for (uint8_t i = 0; i < TX_BufDepth; ++i) {
            freeQueue_Tx.push(i);
            freeQueue_Rx.push(i);
        }
    }

private:
    UART_HandleTypeDef* HUart = nullptr;
    /*  transmit port variable define */
    struct TxMessage {
        uint8_t bufferIndex;
        uint16_t length;
    };
    // 静态内存池：注意，如果是 Cortex-M7 (如 H7/F7)，需放在 Non-Cacheable 区域
    alignas(32) etl::array<etl::array<char, TX_BufferSize>, TX_BufDepth> buffers_Tx;
    SafeQueue<uint8_t, RX_BufDepth> freeQueue_Tx;
    SafeQueue<TxMessage, RX_BufDepth> readyQueue_Tx;
    volatile bool dmaBusy_;
    uint8_t curBufIndex_Tx;
    static LkUart* instance_;   // 静态指针，用于将 C 语言的中断回调路由到 C++ 实例
    /*  receive port variable define */
    SafeQueue<uint8_t, RX_BufDepth> freeQueue_Rx;
    SafeQueue<uint8_t, RX_BufDepth> readQueue_Rx;
    alignas(32) etl::array<etl::string<RX_BufferSize>, RX_BufDepth> buffers_Rx;
    uint8_t curBufIndex_Rx{};
    //signal config define
    SignalContext RxReceive_cfg{};
private:
    /**
     * @brief try to trigger uart TX lint DMA transfer
     */
    void triggerTx() {
        // 进入临界区，防止与中断冲突
        taskENTER_CRITICAL();
        if (!dmaBusy_) {
            TxMessage msg;
            // 检查是否有等待发送的数据
            if (readyQueue_Tx.pop_normal(msg)) {
                dmaBusy_ = true;
                curBufIndex_Tx = msg.bufferIndex;
                // 启动DMA传输
                HAL_UART_Transmit_DMA(HUart,
                                      reinterpret_cast<uint8_t*>(buffers_Tx[msg.bufferIndex].data()),
                                      msg.length);
            }
        }
        taskEXIT_CRITICAL();
    }

    void handleTxCompleteISR() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 发送完成，归还当前缓冲区给空闲队列
        freeQueue_Tx.push_FromISR(curBufIndex_Tx);
        // Check if there are still any data in the queue.
        TxMessage msg;
        if (readyQueue_Tx.pop_FromISR(msg)) {
            curBufIndex_Tx = msg.bufferIndex;
            // 立即开启下一次DMA传输
            HAL_UART_Transmit_DMA(HUart,
                                  reinterpret_cast<uint8_t*>(buffers_Tx[msg.bufferIndex].data()),
                                  msg.length);
        } else {
            // 没有数据了，释放总线
            dmaBusy_ = false;
        }
        // 触发任务调度（如果唤醒了更高优先级任务）
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

public:
    /**
     * @brief Print to Uart
     * @tparam Args
     * @param fmt
     * @param args
     */
    template<typename... Args>
    void print(etl::format_string<Args...> fmt, Args&&... args) {
        uint8_t bufIdx;
        // 1. 获取一个空闲缓冲区 (可设置超时时间，这里设为0，即时获取,没有则跳过)
        if (freeQueue_Tx.pop(bufIdx)) {
            auto &buf = buffers_Tx[bufIdx];
            auto result = etl::format_to_n(buf.data(), TX_BufferSize, fmt, etl::forward<Args>(args)...);
            auto len = static_cast<uint16_t>(result-buf.data());
            if (len > TX_BufferSize) len = TX_BufferSize;
            // 如果使用带Cache的MCU(H7/F7)，这里需要 Clean D-Cache:
            // SCB_CleanDCache_by_Addr((uint32_t*)buf.data(), len);
            //Push the buffered data information with the data onto the ready queue
            TxMessage msg = {bufIdx, len};
            readyQueue_Tx.push(msg);
            triggerTx();
        }
        else{

        }
    }

    /**
     * @brief
     */
    bool Start_DMAIT_Receive(){
        if(freeQueue_Rx.pop(curBufIndex_Rx)){
            HAL_UARTEx_ReceiveToIdle_DMA(HUart, reinterpret_cast<uint8_t*>(buffers_Rx[curBufIndex_Rx].data()), RX_BufferSize); //start DMA receive
            return true;
        }
        else{
            return false;
        }

    }

    /**
     * @brief
     * @param signal
     */
    template<typename SignalPtr>
    bool bindReactor(SignalPtr signalFunc, TaskHandle_t task, uint32_t bitMask){
        if constexpr (std::is_same_v<SignalPtr, decltype(&LkUart::signal_RxComplete)>) {
            if (signalFunc == &LkUart::signal_RxComplete) {
                RxReceive_cfg.task_h = task;
                RxReceive_cfg.bitMask = bitMask;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief
     * @return
     */
    bool getReadRxBuf_empty(){
        return readQueue_Rx.empty();
    }

    /**
     * @brief 供外部中断回调调用的静态接口
     * @param huart
     */
    static void isrTxComplete(UART_HandleTypeDef* huart) {
        if (instance_ && instance_->HUart == huart) {
            instance_->handleTxCompleteISR();
        }
    }

    static void isRxComplete(UART_HandleTypeDef* huart,  uint16_t size) {
        if (instance_ && instance_->HUart == huart) {
            instance_->RxCpltCallback_InISR(size);
        }
    }

    /**
    * @brief 接收一个完整数据帧中断回调（在 HAL_UARTEx_RxEventCallback 中调用）
    * @param Size
    */
    void RxCpltCallback_InISR(uint16_t size) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t bufIdx;
    // If freeQueue is empty this receiving will cover old index of RxBuffer.
    if(freeQueue_Rx.pop_FromISR(bufIdx)){
        readQueue_Rx.push_FromISR(curBufIndex_Rx);
        buffers_Rx[curBufIndex_Rx].uninitialized_resize(size);  //重新刷新大小
        curBufIndex_Rx = bufIdx;
        //emit
        emitFromISR(RxReceive_cfg,&xHigherPriorityTaskWoken);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(HUart, reinterpret_cast<uint8_t*>(buffers_Rx[curBufIndex_Rx].data()), RX_BufferSize); //start DMA receive
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /**
     * @brief
     * @param slot
     */
    void signal_RxComplete(std::function<void(etl::string<RX_BufferSize>&)> slot){
        while(!readQueue_Rx.empty()){
            uint8_t bufIdx;
            readQueue_Rx.pop(bufIdx);
            slot(buffers_Rx[bufIdx]);
            freeQueue_Rx.push(bufIdx);
        }
    }
};

//using LUart = LkUart<>;

// 静态成员初始化
template<size_t TX_BufferSize, size_t TX_BufDepth, size_t RX_BufferSize, size_t RX_BufDepth>
LkUart<TX_BufferSize, TX_BufDepth,RX_BufferSize,RX_BufDepth>* LkUart<TX_BufferSize, TX_BufDepth,RX_BufferSize,RX_BufDepth>::instance_ = nullptr;

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    LkUart<>::isrTxComplete(huart);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    LkUart<>::isRxComplete(huart, Size);
}

#endif //__LKUART_HPP
