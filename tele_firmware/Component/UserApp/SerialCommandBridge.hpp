#pragma once

#include <array>
#include <cstdint>

#include "BridgeCommandParser.hpp"
#include "LkUart.hpp"
#include "RemoteCommandCodec.hpp"
#include "main.h"

class SerialCommandBridge final {
public:
    explicit SerialCommandBridge(UART_HandleTypeDef* uart) noexcept : uart_(uart) {}
    [[nodiscard]] bool start() noexcept;
    void onReceiveFromIsr() noexcept;
    void onErrorFromIsr() noexcept;
    void service(LkUart& log, bool radio_ready = true);
    [[nodiscard]] bool take(RemoteCommandCodec::Payload& payload) noexcept;
    void clear() noexcept;

private:
    static constexpr std::uint16_t ring_size = 128U;
    UART_HandleTypeDef* uart_;
    std::uint8_t incoming_ = 0U;
    std::array<std::uint8_t, ring_size> bytes_{};
    volatile std::uint16_t head_ = 0U;
    volatile std::uint16_t tail_ = 0U;
    volatile bool receive_error_ = false;
    volatile bool restart_required_ = false;
    BridgeCommandParser parser_;
    std::array<RemoteCommandCodec::Payload, 4U> pending_{};
    std::array<std::uint32_t, 4U> queued_at_{};
    std::uint8_t next_ = 0U;
    std::uint8_t count_ = 0U;
};
