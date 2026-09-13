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
    static_assert(TX_BufDepth > 0 && TX_BufDepth <= 255 && RX_BufDepth > 1 && RX_BufDepth <= 255);
    static_assert(RX_BufferSize > 0 && RX_BufferSize <= UINT16_MAX);
public:
    //receive Message type
    explicit LkUart(UART_HandleTypeDef* huart)
            : HUart(huart), dmaBusy_(false), curBufIndex_Tx(0),curBufIndex_Rx(0)
    {
        instance_ = this;
        //Put the indices of all buffers into the free queue
        for (uint8_t i = 0; i < TX_BufDepth; ++i) {
            freeQueue_Tx.push(i);
        }
        for (uint8_t i = 0; i < RX_BufDepth; ++i) {
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
    SafeQueue<uint8_t, TX_BufDepth> freeQueue_Tx;
    SafeQueue<TxMessage, TX_BufDepth> readyQueue_Tx;
    volatile bool dmaBusy_;
    uint8_t curBufIndex_Tx;
    static LkUart* instance_;   // 静态指针，用于将 C 语言的中断回调路由到 C++ 实例
    /*  receive port variable define */
    SafeQueue<uint8_t, RX_BufDepth> freeQueue_Rx;
    SafeQueue<uint8_t, RX_BufDepth> readQueue_Rx;
    alignas(32) etl::array<etl::string<RX_BufferSize>, RX_BufDepth> buffers_Rx;
    uint8_t curBufIndex_Rx{};
    bool rx_started_{};
    bool rx_gap_pending_{};
    etl::array<bool, RX_BufDepth> rx_gap_before_{};
    volatile uint32_t rx_error_count_{};
    volatile uint32_t rx_drop_count_{};
    volatile uint32_t rx_restart_count_{};
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
                if (HAL_UART_Transmit_DMA(HUart,
                                      reinterpret_cast<uint8_t*>(buffers_Tx[msg.bufferIndex].data()),
                                      msg.length) != HAL_OK) {
                    freeQueue_Tx.push_normal(msg.bufferIndex);
                    dmaBusy_ = false;
                }
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
            if (HAL_UART_Transmit_DMA(HUart,
                                  reinterpret_cast<uint8_t*>(buffers_Tx[msg.bufferIndex].data()),
                                  msg.length) != HAL_OK) {
                freeQueue_Tx.push_FromISR(msg.bufferIndex);
                dmaBusy_ = false;
            }
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
        if (rx_started_) { return true; }
        if(freeQueue_Rx.pop(curBufIndex_Rx)){
            if (restart_receive()) {
                rx_started_ = true;
                return true;
            }
            freeQueue_Rx.push(curBufIndex_Rx);
            return false;
        }
        else{
            return false;
        }

    }

    // Retry a failed rearm outside the ISR, without polling for a module/peer.
    void service_receive() {
        taskENTER_CRITICAL();
        if (rx_started_ && HUart->RxState == HAL_UART_STATE_READY) {
            (void)restart_receive();
        }
        taskEXIT_CRITICAL();
    }

    /**
     * @brief 绑定订阅任务
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

    static void isrError(UART_HandleTypeDef* huart) {
        if (instance_ && instance_->HUart == huart && instance_->rx_started_) {
            instance_->rx_error_count_ = instance_->rx_error_count_ + 1U;
            instance_->rx_gap_pending_ = true;
            // The HAL completes/aborts DMA before invoking the error callback.
            if (huart->RxState == HAL_UART_STATE_READY) {
                (void)instance_->restart_receive();
            }
        }
    }

    /**
    * @brief 接收一个完整数据帧中断回调（在 HAL_UARTEx_RxEventCallback 中调用）
    * @param Size
    */
    void RxCpltCallback_InISR(uint16_t size) {
        // A DMA half-transfer is not an idle frame; DMA still owns this buffer.
        if (!rx_started_ || HAL_UARTEx_GetRxEventType(HUart) == HAL_UART_RXEVENT_HT) { return; }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t next;
        if (size > 0U && size <= RX_BufferSize && freeQueue_Rx.pop_FromISR(next)) {
            buffers_Rx[curBufIndex_Rx].uninitialized_resize(size);
            rx_gap_before_[curBufIndex_Rx] = rx_gap_pending_;
            if (readQueue_Rx.push_FromISR(curBufIndex_Rx)) {
                curBufIndex_Rx = next;
                rx_gap_pending_ = false;
                emitFromISR(RxReceive_cfg, &xHigherPriorityTaskWoken);
            } else {
                freeQueue_Rx.push_FromISR(next);
                mark_receive_drop();
            }
        } else {
            mark_receive_drop();
        }
        (void)restart_receive();
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    /**
     * @brief
     * @param slot
     */
    void signal_RxComplete(std::function<void(etl::string<RX_BufferSize>&)> slot){
        for (size_t count = 0U; count < RX_BufDepth; ++count) {
            uint8_t bufIdx;
            if (!readQueue_Rx.pop(bufIdx)) { break; }
            if (rx_gap_before_[bufIdx]) {
                etl::string<RX_BufferSize> gap;
                slot(gap);
            }
            slot(buffers_Rx[bufIdx]);
            freeQueue_Rx.push(bufIdx);
        }
        if (!readQueue_Rx.empty() && RxReceive_cfg.task_h != nullptr) {
            xTaskNotify(RxReceive_cfg.task_h, RxReceive_cfg.bitMask, eSetBits);
        }
    }

private:
    void mark_receive_drop() {
        rx_drop_count_ = rx_drop_count_ + 1U;
        rx_gap_pending_ = true;
    }

    bool restart_receive() {
        const auto result = HAL_UARTEx_ReceiveToIdle_DMA(HUart,
            reinterpret_cast<uint8_t*>(buffers_Rx[curBufIndex_Rx].data()), RX_BufferSize);
        if (result != HAL_OK) {
            if (rx_started_) { rx_gap_pending_ = true; }
            return false;
        }
        // Receive-to-idle enables HT by default; only IDLE/TC hand off a buffer.
        (void)__HAL_DMA_DISABLE_IT(HUart->hdmarx, DMA_IT_HT);
        rx_restart_count_ = rx_restart_count_ + 1U;
        return true;
    }
};

//using LUart = LkUart<>;

// 静态成员初始化
template<size_t TX_BufferSize, size_t TX_BufDepth, size_t RX_BufferSize, size_t RX_BufDepth>
LkUart<TX_BufferSize, TX_BufDepth,RX_BufferSize,RX_BufDepth>* LkUart<TX_BufferSize, TX_BufDepth,RX_BufferSize,RX_BufDepth>::instance_ = nullptr;

#endif //__LKUART_HPP
