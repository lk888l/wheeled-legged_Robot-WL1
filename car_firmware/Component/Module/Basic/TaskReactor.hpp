#pragma once

#include <array>
#include <atomic>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>
#include <system_error>
#include <utility>

#include "FreeRTOS.h"
#include "task.h"

#include "BasicObject.hpp"
#include "etl/string_view.h"

class TaskReactor : public BasicObject
{
public:
    using SlotCallback = std::function<void()>;

    struct strCMD_t
    {
        etl::string_view command;
        etl::string_view args;
    };

    explicit TaskReactor(TaskHandle_t task_handle = nullptr)
        : reactor_task_(task_handle != nullptr ? task_handle : xTaskGetCurrentTaskHandle())
    {
    }

    void bindToCurrentTask()
    {
        reactor_task_ = xTaskGetCurrentTaskHandle();
    }

    [[nodiscard]] TaskHandle_t taskHandle() const
    {
        return reactor_task_;
    }

    void stop()
    {
        stop_requested_.store(true, std::memory_order_release);
        if (reactor_task_ != nullptr)
        {
            xTaskNotify(reactor_task_, stop_bit_, eSetBits);
        }
    }

    [[nodiscard]] bool stopped() const
    {
        return stop_requested_.load(std::memory_order_acquire);
    }

    bool setSlot(std::uint32_t bit_mask, SlotCallback callback)
    {
        if (bit_mask == 0 || (bit_mask & (bit_mask - 1U)) != 0)
        {
            return false;
        }

        const int index = __builtin_ctz(bit_mask);
        if (index < 0 || index >= static_cast<int>(slots_.size() - 1))
        {
            return false;
        }

        slots_[static_cast<std::size_t>(index)] = std::move(callback);
        return true;
    }

    template <typename Sender, typename SignalMethod, typename SlotLambda>
    bool connect(Sender* sender, SignalMethod signal, SlotLambda slot)
    {
        if (sender == nullptr)
        {
            return false;
        }

        const std::uint32_t bit_mask = allocateNotifyBit();
        if (bit_mask == 0)
        {
            return false;
        }

        if (!sender->bindReactor(signal, reactor_task_, bit_mask))
        {
            return false;
        }

        return setSlot(bit_mask, [sender, signal, slot]() {
            static_cast<void>((sender->*signal)(slot));
        });
    }

    void taskLoop(TickType_t ticks_to_wait = portMAX_DELAY,
                  const SlotCallback& after_notify = nullptr,
                  const SlotCallback& on_timeout = nullptr)
    {
        bindToCurrentTask();
        stop_requested_.store(false, std::memory_order_release);

        std::uint32_t notified_value = 0;
        TickType_t last_wake_time = xTaskGetTickCount();

        while (!stop_requested_.load(std::memory_order_acquire))
        {
            TickType_t ticks_remaining = portMAX_DELAY;
            if (on_timeout && ticks_to_wait != portMAX_DELAY)
            {
                const TickType_t elapsed = xTaskGetTickCount() - last_wake_time;
                if (elapsed >= ticks_to_wait)
                {
                    on_timeout();
                    // A timeout callback may synchronously produce work (for
                    // example, polling a level-held peripheral IRQ). Process
                    // that work before blocking for another timeout period.
                    if (after_notify)
                    {
                        after_notify();
                    }
                    last_wake_time = xTaskGetTickCount();
                    ticks_remaining = ticks_to_wait;
                }
                else
                {
                    ticks_remaining = ticks_to_wait - elapsed;
                }
            }

            if (xTaskNotifyWait(0, 0xFFFFFFFFU, &notified_value, ticks_remaining) == pdTRUE)
            {
                if ((notified_value & stop_bit_) != 0)
                {
                    stop_requested_.store(true, std::memory_order_release);
                }
                dispatch(notified_value & ~stop_bit_);
                if (after_notify)
                {
                    after_notify();
                }
            }
        }
    }

    static bool parseStrCMD(etl::string_view input, strCMD_t& command)
    {
        const std::size_t start = input.find_first_not_of(' ');
        if (start == etl::string_view::npos)
        {
            command = {};
            return false;
        }
        input.remove_prefix(start);

        const std::size_t space_position = input.find(' ');
        if (space_position == etl::string_view::npos)
        {
            command.command = input;
            command.args = {};
            return true;
        }

        command.command = input.substr(0, space_position);
        command.args = input.substr(space_position + 1);
        const std::size_t first_argument = command.args.find_first_not_of(' ');
        if (first_argument == etl::string_view::npos)
        {
            command.args = {};
        }
        else
        {
            command.args.remove_prefix(first_argument);
        }
        return true;
    }

    template <typename T>
    static bool parseStrArg(etl::string_view& arguments, T& value)
    {
        const std::size_t first = arguments.find_first_not_of(' ');
        if (first == etl::string_view::npos)
        {
            return false;
        }
        arguments.remove_prefix(first);

        const std::size_t last = arguments.find(' ');
        const etl::string_view token = arguments.substr(0, last);
        const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
        if (result.ec != std::errc{} || result.ptr != token.data() + token.size())
        {
            return false;
        }

        if (last == etl::string_view::npos)
        {
            arguments = {};
        }
        else
        {
            arguments.remove_prefix(last + 1);
        }
        return true;
    }

private:
    [[nodiscard]] std::uint32_t allocateNotifyBit() const
    {
        for (std::size_t index = 0; index < slots_.size() - 1; ++index)
        {
            if (!slots_[index])
            {
                return 1UL << index;
            }
        }
        return 0;
    }

    void dispatch(std::uint32_t notified_value)
    {
        for (std::size_t index = 0; notified_value != 0 && index < slots_.size() - 1; ++index)
        {
            const std::uint32_t bit = 1UL << index;
            if ((notified_value & bit) != 0)
            {
                if (slots_[index])
                {
                    slots_[index]();
                }
                notified_value &= ~bit;
            }
        }
    }

    static constexpr std::uint32_t stop_bit_ = 1UL << 31;

    TaskHandle_t reactor_task_ = nullptr;
    std::array<SlotCallback, 32> slots_{};
    std::atomic<bool> stop_requested_{false};
};
