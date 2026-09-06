#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

// Task-owned framing/parser. DMA chunks are not command boundaries; CR/LF or
// an application idle timeout terminates a command from a serial assistant.
class SerialCommandQueue final {
public:
    enum class Kind { transmit, joystick_on, joystick_off, help, status, invalid, too_long };
    struct Command {
        Kind kind = Kind::invalid;
        std::array<std::uint8_t, 32U> payload{};
    };

    void append(std::string_view bytes) noexcept;
    void finish() noexcept;
    void discardPartial() noexcept;
    [[nodiscard]] bool pop(Command& command) noexcept;
    [[nodiscard]] std::uint32_t droppedCommands() const noexcept { return dropped_; }

private:
    void enqueue(const Command& command) noexcept;

    std::array<char, 96U> line_{};
    std::size_t length_ = 0U;
    bool invalid_ = false;
    bool too_long_ = false;
    std::array<Command, 8U> commands_{};
    std::size_t head_ = 0U;
    std::size_t tail_ = 0U;
    std::size_t count_ = 0U;
    std::uint32_t dropped_ = 0U;
};
