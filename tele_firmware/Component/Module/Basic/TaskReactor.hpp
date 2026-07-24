/********************************************************************************
  * @file           : TaskReactor.hpp
  * @brief          : FreeRTOS task-notification signal reactor.
  *******************************************************************************/

#ifndef TASKREACTOR_HPP
#define TASKREACTOR_HPP

#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string_view>
#include <system_error>
#include <type_traits>

#include "etl/string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "BasicObject.hpp"

class TaskReactor : public BasicObject {
    using SlotCallback = std::function<void()>;

public:
    TaskReactor()
        : reactorTask_(xTaskGetCurrentTaskHandle()) {}

    struct strCMD_t {
        etl::string_view command;
        etl::string_view args;
    };

    template<typename Sender, typename SignalMethod, typename SlotLambda>
    bool connect(Sender* sender, SignalMethod signal, SlotLambda slot) {
        constexpr bool use_static_binding =
            std::is_convertible_v<SlotLambda, void (*)()> &&
            std::is_trivially_copyable_v<SignalMethod> &&
            sizeof(SignalMethod) <= STATIC_SIGNAL_STORAGE_BYTES;

        if constexpr (use_static_binding) {
            if (static_binding_count_ >= static_bindings_.size()) {
                return false;
            }
        }

        const std::uint32_t bitMask = allocNotifyBit();
        if (bitMask == 0U) {
            return false;
        }
        if (!sender->bindReactor(signal, reactorTask_, bitMask)) {
            return false;
        }

        const auto bitIndex = static_cast<std::uint8_t>(__builtin_ctz(bitMask));

        // Captureless button handlers can be represented as plain function
        // pointers. Keep their binding in fixed storage so connecting the GPIO
        // task never touches the small newlib heap while the NRF task is
        // starting and waiting for SPI DMA completions.
        if constexpr (use_static_binding) {
            auto& binding = static_bindings_[static_binding_count_++];
            binding.sender = sender;
            binding.slot = static_cast<void (*)()>(slot);
            std::memcpy(binding.signal.data(), &signal, sizeof(signal));
            binding.invoke = [](StaticBinding& stored) {
                SignalMethod stored_signal{};
                std::memcpy(&stored_signal, stored.signal.data(), sizeof(stored_signal));
                const std::function<void()> callback(stored.slot);
                (static_cast<Sender*>(stored.sender)->*stored_signal)(callback);
            };
            slots_[bitIndex] = [binding_ptr = &binding]() {
                binding_ptr->invoke(*binding_ptr);
            };
            return true;
        } else {
            slots_[bitIndex] = [sender, signal, slot]() {
                (sender->*signal)(slot);
            };
            return true;
        }
    }

    inline void taskLoop(
        TickType_t ticksToWait = portMAX_DELAY,
        const std::function<void()>& afterNotify = nullptr,
        const std::function<void()>& periodic = nullptr) {
        std::uint32_t notifiedValue = 0U;
        TickType_t lastWakeTime = xTaskGetTickCount();

        while (true) {
            const TickType_t now = xTaskGetTickCount();
            TickType_t ticksRemaining = portMAX_DELAY;

            if (periodic && ticksToWait != portMAX_DELAY) {
                TickType_t elapsed = now - lastWakeTime;
                if (elapsed >= ticksToWait) {
                    periodic();
                    lastWakeTime = xTaskGetTickCount();
                    elapsed = 0U;
                }
                ticksRemaining = ticksToWait - elapsed;
            }

            if (xTaskNotifyWait(0x00U, 0xFFFFFFFFU, &notifiedValue, ticksRemaining) == pdTRUE) {
                for (std::uint8_t i = 0U; notifiedValue != 0U && i < slots_.size(); ++i) {
                    const std::uint32_t mask = (1UL << i);
                    if ((notifiedValue & mask) != 0U) {
                        if (slots_[i]) {
                            slots_[i]();
                        }
                        notifiedValue &= ~mask;
                    }
                }
                if (afterNotify) {
                    afterNotify();
                }
            }
        }
    }

    static bool parseStrCMD(etl::string_view input, strCMD_t& strCmd) {
        const auto start = input.find_first_not_of(" ");
        if (start == etl::string_view::npos) {
            strCmd.command = "";
            strCmd.args = "";
            return false;
        }
        input.remove_prefix(start);

        const auto spacePos = input.find(' ');
        if (spacePos == etl::string_view::npos) {
            strCmd.command = input;
            strCmd.args = "";
            return true;
        }

        strCmd.command = input.substr(0U, spacePos);
        auto args = input.substr(spacePos + 1U);
        const auto firstArg = args.find_first_not_of(" ");
        if (firstArg == etl::string_view::npos) {
            strCmd.args = "";
        } else {
            args.remove_prefix(firstArg);
            strCmd.args = args;
        }
        return true;
    }

    template<typename T>
    static bool parseStrArg(etl::string_view& strArg, T& value) {
        const auto first = strArg.find_first_not_of(" ");
        if (first == etl::string_view::npos) {
            strArg = "";
            return false;
        }
        strArg.remove_prefix(first);

        const auto last = strArg.find(' ');
        const etl::string_view token = strArg.substr(0U, last);
        const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
        if (last == etl::string_view::npos) {
            strArg = "";
        } else {
            strArg.remove_prefix(last + 1U);
        }
        return result.ec == std::errc();
    }

private:
    static constexpr std::size_t STATIC_BINDING_CAPACITY = 4U;
    static constexpr std::size_t STATIC_SIGNAL_STORAGE_BYTES = 16U;

    struct StaticBinding {
        alignas(std::max_align_t)
        std::array<std::byte, STATIC_SIGNAL_STORAGE_BYTES> signal{};
        void* sender = nullptr;
        void (*slot)() = nullptr;
        void (*invoke)(StaticBinding&) = nullptr;
    };

    std::uint32_t allocNotifyBit() const {
        for (std::uint8_t i = 0U; i < slots_.size(); ++i) {
            if (!slots_[i]) {
                return (1UL << i);
            }
        }
        return 0U;
    }

    std::array<SlotCallback, 32> slots_{};
    std::array<StaticBinding, STATIC_BINDING_CAPACITY> static_bindings_{};
    std::uint8_t static_binding_count_ = 0U;
    TaskHandle_t reactorTask_ = nullptr;
};

#endif // TASKREACTOR_HPP
