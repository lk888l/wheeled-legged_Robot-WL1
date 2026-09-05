#include "LkUart.hpp"

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

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    LkUart::handleTxCompleteFromIsr(uart);
}
