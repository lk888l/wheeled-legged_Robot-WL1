#include "LkUart.hpp"

#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

LkUart* LkUart::instance_ = nullptr;

LkUart::LkUart(UART_HandleTypeDef* uart) noexcept
    : uart_(uart)
{
    instance_ = this;
}

bool LkUart::acquireBuffer(std::uint8_t& index) noexcept
{
    taskENTER_CRITICAL();
    for (std::uint8_t candidate = 0U; candidate < buffer_count; ++candidate) {
        const std::uint8_t bit = static_cast<std::uint8_t>(1U << candidate);
        if ((free_mask_ & bit) != 0U) {
            free_mask_ = static_cast<std::uint8_t>(free_mask_ & ~bit);
            index = candidate;
            taskEXIT_CRITICAL();
            return true;
        }
    }
    taskEXIT_CRITICAL();
    return false;
}

void LkUart::releaseBuffer(std::uint8_t index) noexcept
{
    taskENTER_CRITICAL();
    free_mask_ = static_cast<std::uint8_t>(free_mask_ | (1U << index));
    taskEXIT_CRITICAL();
}

void LkUart::noteDroppedMessage() noexcept
{
    taskENTER_CRITICAL();
    ++dropped_messages_;
    taskEXIT_CRITICAL();
}

std::uint32_t LkUart::droppedMessages() const noexcept
{
    taskENTER_CRITICAL();
    const std::uint32_t result = dropped_messages_;
    taskEXIT_CRITICAL();
    return result;
}

bool LkUart::enqueue(const Message& message) noexcept
{
    taskENTER_CRITICAL();
    if (ready_count_ >= ready_.size()) {
        taskEXIT_CRITICAL();
        return false;
    }

    ready_[ready_tail_] = message;
    ready_tail_ = static_cast<std::uint8_t>((ready_tail_ + 1U) % ready_.size());
    ++ready_count_;
    startNextTransferLocked();
    taskEXIT_CRITICAL();
    return true;
}

void LkUart::startNextTransferLocked() noexcept
{
    while (!dma_busy_ && ready_count_ > 0U) {
        const Message message = ready_[ready_head_];
        ready_head_ = static_cast<std::uint8_t>((ready_head_ + 1U) % ready_.size());
        --ready_count_;

        active_buffer_ = message.buffer_index;
        if (HAL_UART_Transmit_DMA(
                uart_,
                reinterpret_cast<std::uint8_t*>(buffers_[active_buffer_].data()),
                message.length) == HAL_OK) {
            dma_busy_ = true;
        } else {
            free_mask_ = static_cast<std::uint8_t>(
                free_mask_ | (1U << active_buffer_));
            ++dropped_messages_;
        }
    }
}

void LkUart::completeTransferFromIsr() noexcept
{
    const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
    if (dma_busy_) {
        free_mask_ = static_cast<std::uint8_t>(
            free_mask_ | (1U << active_buffer_));
        dma_busy_ = false;
        startNextTransferLocked();
    }
    taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
}

void LkUart::handleTxCompleteFromIsr(UART_HandleTypeDef* uart) noexcept
{
    if (instance_ != nullptr && instance_->uart_ == uart) {
        instance_->completeTransferFromIsr();
    }
}

bool LkUart::armReceive() noexcept
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(
            uart_, rx_dma_.data(), static_cast<std::uint16_t>(rx_dma_.size())) != HAL_OK) {
        return false;
    }
    // HT is not a completed buffer in normal DMA mode.
    __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
    return true;
}

bool LkUart::startReceive(TaskHandle_t receiver) noexcept
{
    taskENTER_CRITICAL();
    receiver_ = receiver;
    const bool success = uart_->RxState == HAL_UART_STATE_BUSY_RX || armReceive();
    taskEXIT_CRITICAL();
    return success;
}

bool LkUart::readReceived(ReceivedData& data) noexcept
{
    taskENTER_CRITICAL();
    if (rx_count_ == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }
    data = rx_ready_[rx_head_];
    rx_head_ = static_cast<std::uint8_t>((rx_head_ + 1U) % rx_ready_.size());
    --rx_count_;
    taskEXIT_CRITICAL();
    return true;
}

std::uint32_t LkUart::receiveErrors() const noexcept
{
    taskENTER_CRITICAL();
    const std::uint32_t errors = rx_errors_;
    taskEXIT_CRITICAL();
    return errors;
}

void LkUart::notifyReceiverFromIsr() noexcept
{
    if (receiver_ != nullptr) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(receiver_, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void LkUart::receiveFromIsr(std::uint16_t size) noexcept
{
    if (HAL_UARTEx_GetRxEventType(uart_) == HAL_UART_RXEVENT_HT) {
        return;
    }
    if (size > 0U && size <= rx_dma_.size() && rx_count_ < rx_ready_.size()) {
        ReceivedData& received = rx_ready_[rx_tail_];
        std::copy_n(rx_dma_.begin(), size, received.data.begin());
        received.length = size;
        rx_tail_ = static_cast<std::uint8_t>((rx_tail_ + 1U) % rx_ready_.size());
        ++rx_count_;
    } else {
        ++rx_errors_;
    }
    // Copy before rearming: DMA must never overwrite a queued command.
    if (!armReceive()) {
        ++rx_errors_;
    }
    notifyReceiverFromIsr();
}

void LkUart::handleRxEventFromIsr(UART_HandleTypeDef* uart, std::uint16_t size) noexcept
{
    if (instance_ != nullptr && instance_->uart_ == uart) {
        instance_->receiveFromIsr(size);
    }
}

void LkUart::handleErrorFromIsr(UART_HandleTypeDef* uart) noexcept
{
    if (instance_ == nullptr || instance_->uart_ != uart) {
        return;
    }
    ++instance_->rx_errors_;
    // A framing/overrun error aborts RX only. Do not release an active TX buffer.
    if (uart->gState == HAL_UART_STATE_READY) {
        instance_->completeTransferFromIsr();
    }
    // The task rearms RX after HAL finishes its DMA abort callback.
    instance_->notifyReceiverFromIsr();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    LkUart::handleTxCompleteFromIsr(uart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    LkUart::handleErrorFromIsr(uart);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, std::uint16_t size)
{
    LkUart::handleRxEventFromIsr(uart, size);
}
