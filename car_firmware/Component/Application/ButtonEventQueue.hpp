#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "Button.hpp"

namespace app {

struct ButtonEvent {
    Button::Event type{Button::Event::none};
    uint32_t tick{0U};
};

// One producer (ButtonTask), one consumer (CommandServiceTask). No interrupt
// masking, allocation, spinning, blocking, or task notification. A full queue
// drops the NEW event and counts it; earlier events retain their order.
class ButtonEventQueue final {
public:
    static constexpr uint32_t capacity = 8U;
    static_assert(std::atomic<uint32_t>::is_always_lock_free);

    [[nodiscard]] bool push(ButtonEvent event) noexcept
    {
        const uint32_t write = write_.load(std::memory_order_relaxed);
        if (uint32_t(write - read_.load(std::memory_order_acquire)) == capacity) {
            dropped_.store(dropped_.load(std::memory_order_relaxed) + 1U,
                           std::memory_order_relaxed);
            return false;
        }
        events_[write % capacity] = event;
        write_.store(write + 1U, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool try_pop(ButtonEvent& event) noexcept
    {
        const uint32_t read = read_.load(std::memory_order_relaxed);
        if (read == write_.load(std::memory_order_acquire)) {
            return false;
        }
        event = events_[read % capacity];
        read_.store(read + 1U, std::memory_order_release);
        return true;
    }

    [[nodiscard]] uint32_t dropped() const noexcept
    {
        return dropped_.load(std::memory_order_relaxed);
    }

private:
    std::array<ButtonEvent, capacity> events_{};
    std::atomic<uint32_t> write_{0U};
    std::atomic<uint32_t> read_{0U};
    std::atomic<uint32_t> dropped_{0U};
};

} // namespace app
