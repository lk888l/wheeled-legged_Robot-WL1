#include "SerialCommandBridge.hpp"

#include <algorithm>
#include "FreeRTOS.h"
#include "task.h"

bool SerialCommandBridge::start() noexcept
{
    // A byte interrupt keeps framing independent from USB/UART idle gaps.
    // At <= 8 commands/s this is at most 256 RX interrupts/s.
    return HAL_UART_Receive_IT(uart_, &incoming_, 1U) == HAL_OK;
}

void SerialCommandBridge::onReceiveFromIsr() noexcept
{
    // HAL can call RxCplt before ErrorCallback for the same damaged byte.
    // Do not enqueue it or clear HAL's error by rearming reception here.
    if (uart_->ErrorCode != HAL_UART_ERROR_NONE) {
        onErrorFromIsr();
        return;
    }
    const auto next = static_cast<std::uint16_t>((head_ + 1U) % ring_size);
    if (receive_error_ || next == tail_) {
        receive_error_ = true;
    } else {
        bytes_[head_] = incoming_;
        head_ = next;
    }
    if (!start()) { restart_required_ = true; }
}

void SerialCommandBridge::onErrorFromIsr() noexcept
{
    receive_error_ = true;
    restart_required_ = true;
}

void SerialCommandBridge::service(LkUart& log, bool radio_ready)
{
    // Snapshot interrupt-owned state before parsing or logging in task context.
    taskENTER_CRITICAL();
    const bool damaged = receive_error_;
    receive_error_ = false;
    const bool restart = restart_required_;
    restart_required_ = false;
    if (damaged) { tail_ = head_; }
    taskEXIT_CRITICAL();
    if (damaged) {
        parser_.discardLine();
        clear();
        log.print("bridge: rejected UART receive error; resend full line\n");
    }
    if (restart && uart_->RxState == HAL_UART_STATE_READY && !start()) {
        restart_required_ = true;
    }

    for (std::uint16_t n = 0U; n < ring_size; ++n) {
        taskENTER_CRITICAL();
        if (receive_error_) {
            tail_ = head_;
            taskEXIT_CRITICAL();
            parser_.discardLine();
            clear();
            return;
        }
        if (tail_ == head_) { taskEXIT_CRITICAL(); break; }
        const auto byte = bytes_[tail_];
        tail_ = static_cast<std::uint16_t>((tail_ + 1U) % ring_size);
        taskEXIT_CRITICAL();
        const auto result = parser_.feed(byte);
        if (result == BridgeCommandParser::Result::rejected) {
            log.print("bridge: rejected command (PID/anglebias only, max 31 ASCII bytes)\n");
        } else if (result == BridgeCommandParser::Result::accepted) {
            if (!radio_ready) {
                log.print("bridge: rejected radio unavailable\n");
                continue;
            }
            if (count_ == pending_.size()) {
                log.print("bridge: rejected queue full\n");
                continue;
            }
            const auto index = static_cast<std::uint8_t>((next_ + count_) % pending_.size());
            auto& payload = pending_[index];
            payload.fill(0U);
            const auto command = parser_.completed();
            std::copy(command.begin(), command.end(), payload.begin());
            queued_at_[index] = xTaskGetTickCount();
            ++count_;
            log.print("bridge: queued {}\n", reinterpret_cast<const char*>(payload.data()));
        }
    }
    if (!radio_ready) { parser_.discardPartialLine(); }
    while (count_ > 0U &&
           static_cast<TickType_t>(xTaskGetTickCount() - queued_at_[next_]) > pdMS_TO_TICKS(500U)) {
        next_ = static_cast<std::uint8_t>((next_ + 1U) % pending_.size());
        --count_;
        log.print("bridge: rejected expired command\n");
    }
}

bool SerialCommandBridge::take(RemoteCommandCodec::Payload& payload) noexcept
{
    taskENTER_CRITICAL();
    // An RX error may arrive after service()'s final byte was parsed.
    if (receive_error_ || count_ == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }
    payload = pending_[next_];
    next_ = static_cast<std::uint8_t>((next_ + 1U) % pending_.size());
    --count_;
    taskEXIT_CRITICAL();
    return true;
}

void SerialCommandBridge::clear() noexcept
{
    next_ = 0U;
    count_ = 0U;
}
