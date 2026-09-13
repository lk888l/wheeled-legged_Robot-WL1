#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

// A serial line may span arbitrarily many UART interrupts. Never forward a
// partial, truncated, binary, or actuator command to the radio.
class BridgeCommandParser final {
public:
    static constexpr std::size_t maximum_length = 31U;
    enum class Result { incomplete, accepted, rejected };

    Result feed(std::uint8_t byte) noexcept
    {
        if (byte == '\n') {
            if (size_ == 0U && !invalid_) {
                carriage_return_ = false;
                return Result::incomplete;
            }
            const bool accepted = !invalid_ && allowed(text());
            completed_size_ = size_;
            size_ = 0U;
            invalid_ = false;
            carriage_return_ = false;
            return accepted ? Result::accepted : Result::rejected;
        }
        if (byte == '\r' && !carriage_return_) {
            carriage_return_ = true;
            return Result::incomplete;
        }
        if (carriage_return_ || byte < 0x20U || byte > 0x7EU || size_ == maximum_length) {
            invalid_ = true;
            return Result::incomplete;
        }
        buffer_[size_++] = static_cast<char>(byte);
        return Result::incomplete;
    }

    void discardLine() noexcept { invalid_ = true; }
    void discardPartialLine() noexcept
    {
        if (size_ != 0U || carriage_return_) { invalid_ = true; }
    }
    [[nodiscard]] std::string_view completed() const noexcept
    {
        return {buffer_.data(), completed_size_};
    }

private:
    [[nodiscard]] std::string_view text() const noexcept { return {buffer_.data(), size_}; }

    static std::string_view token(std::string_view& input) noexcept
    {
        const auto first = input.find_first_not_of(' ');
        if (first == std::string_view::npos) { input = {}; return {}; }
        input.remove_prefix(first);
        const auto end = input.find(' ');
        const auto value = input.substr(0U, end);
        input = end == std::string_view::npos ? std::string_view{} : input.substr(end);
        return value;
    }

    static bool number(std::string_view value) noexcept
    {
        if (value.empty()) { return false; }
        if (value.front() == '-') { value.remove_prefix(1U); }
        bool digits = false;
        while (!value.empty() && value.front() >= '0' && value.front() <= '9') {
            digits = true;
            value.remove_prefix(1U);
        }
        if (!value.empty() && value.front() == '.') {
            value.remove_prefix(1U);
            while (!value.empty() && value.front() >= '0' && value.front() <= '9') {
                digits = true;
                value.remove_prefix(1U);
            }
        }
        if (!digits) { return false; }
        if (!value.empty() && (value.front() == 'e' || value.front() == 'E')) {
            value.remove_prefix(1U);
            if (!value.empty() && (value.front() == '-' || value.front() == '+')) {
                value.remove_prefix(1U);
            }
            bool exponent_digits = false;
            while (!value.empty() && value.front() >= '0' && value.front() <= '9') {
                exponent_digits = true;
                value.remove_prefix(1U);
            }
            if (!exponent_digits) { return false; }
        }
        return value.empty();
    }

    static bool allowed(std::string_view input) noexcept
    {
        const auto command = token(input);
        auto calibration = input;
        if ((command == "anglepid" || command == "anglebias") &&
            token(calibration) == "auto" && token(calibration).empty()) { return true; }
        if (command != "anglebias") {
            if (command != "anglepid" && command != "velocitypid" &&
                command != "differpid" && command != "rollpid") { return false; }
            const auto option = token(input);
            if (option != "-p" && option != "-i" &&
                !(option == "-d" && command != "rollpid")) { return false; }
        }
        const auto value = token(input);
        return number(value) && token(input).empty();
    }

    std::array<char, maximum_length> buffer_{};
    std::size_t size_ = 0U;
    std::size_t completed_size_ = 0U;
    bool invalid_ = false;
    bool carriage_return_ = false;
};
