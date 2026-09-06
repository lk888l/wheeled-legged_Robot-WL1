#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "etl/format.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * Non-blocking UART logger and DMA receiver.
 *
 * Formatting happens in the caller's task, while DMA owns a separate fixed
 * buffer. No heap allocation or logging task is required.
 */
class LkUart final {
public:
    explicit LkUart(UART_HandleTypeDef* uart) noexcept;

    struct ReceivedData {
        std::array<char, 128U> data{};
        std::uint16_t length = 0U;
    };

    [[nodiscard]] bool startReceive(TaskHandle_t receiver) noexcept;
    [[nodiscard]] bool readReceived(ReceivedData& data) noexcept;
    [[nodiscard]] std::uint32_t receiveErrors() const noexcept;

    template<typename... Args>
    void print(etl::format_string<Args...> format, Args&&... args)
    {
        std::uint8_t buffer_index = 0U;
        if (!acquireBuffer(buffer_index)) {
            noteDroppedMessage();
            return;
        }

        Buffer& buffer = buffers_[buffer_index];
        const auto end = etl::format_to_n(
            buffer.data(),
            buffer.size(),
            format,
            etl::forward<Args>(args)...);
        const std::size_t length = static_cast<std::size_t>(end - buffer.data());
        const Message message{
            .buffer_index = buffer_index,
            .length = static_cast<std::uint16_t>(
                length < buffer.size() ? length : buffer.size()),
        };

        if (!enqueue(message)) {
            releaseBuffer(buffer_index);
            noteDroppedMessage();
        }
    }

    [[nodiscard]] std::uint32_t droppedMessages() const noexcept;

    static void handleTxCompleteFromIsr(UART_HandleTypeDef* uart) noexcept;
    static void handleRxEventFromIsr(UART_HandleTypeDef* uart, std::uint16_t size) noexcept;
    static void handleErrorFromIsr(UART_HandleTypeDef* uart) noexcept;

private:
    static constexpr std::size_t buffer_size = 128U;
    static constexpr std::size_t buffer_count = 4U;
    static_assert(buffer_count <= 8U);

    using Buffer = std::array<char, buffer_size>;

    struct Message {
        std::uint8_t buffer_index;
        std::uint16_t length;
    };

    [[nodiscard]] bool acquireBuffer(std::uint8_t& index) noexcept;
    void releaseBuffer(std::uint8_t index) noexcept;
    void noteDroppedMessage() noexcept;
    [[nodiscard]] bool enqueue(const Message& message) noexcept;
    void startNextTransferLocked() noexcept;
    void completeTransferFromIsr() noexcept;
    [[nodiscard]] bool armReceive() noexcept;
    void receiveFromIsr(std::uint16_t size) noexcept;
    void notifyReceiverFromIsr() noexcept;

    static LkUart* instance_;

    UART_HandleTypeDef* uart_;
    std::array<Buffer, buffer_count> buffers_{};
    std::array<Message, buffer_count> ready_{};
    std::uint8_t free_mask_ = (1U << buffer_count) - 1U;
    std::uint8_t ready_head_ = 0U;
    std::uint8_t ready_tail_ = 0U;
    std::uint8_t ready_count_ = 0U;
    std::uint8_t active_buffer_ = 0U;
    bool dma_busy_ = false;
    std::uint32_t dropped_messages_ = 0U;

    std::array<std::uint8_t, 128U> rx_dma_{};
    std::array<ReceivedData, 8U> rx_ready_{};
    std::uint8_t rx_head_ = 0U;
    std::uint8_t rx_tail_ = 0U;
    std::uint8_t rx_count_ = 0U;
    std::uint32_t rx_errors_ = 0U;
    TaskHandle_t receiver_ = nullptr;
};
