#include "SerialCommandQueue.hpp"

#include <algorithm>

namespace {
std::string_view trim(std::string_view text) noexcept
{
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1U);
}
} // namespace

void SerialCommandQueue::append(std::string_view bytes) noexcept
{
    for (const unsigned char byte : bytes) {
        if (byte == '\r' || byte == '\n') {
            finish();
        } else if (byte == '\b' || byte == 0x7FU) {
            if (length_ > 0U) {
                --length_;
            }
        } else if ((byte < 0x20U && byte != '\t') || byte > 0x7EU) {
            invalid_ = true;
        } else if (length_ < line_.size()) {
            line_[length_++] = static_cast<char>(byte);
        } else {
            too_long_ = true;
        }
    }
}

void SerialCommandQueue::finish() noexcept
{
    const std::string_view line = trim({line_.data(), length_});
    if (line.empty() && !invalid_ && !too_long_) {
        length_ = 0U;
        return;
    }

    Command command;
    if (too_long_) {
        command.kind = Kind::too_long;
    } else if (invalid_) {
        command.kind = Kind::invalid;
    } else if (line == "joystick on") {
        command.kind = Kind::joystick_on;
    } else if (line == "joystick off") {
        command.kind = Kind::joystick_off;
    } else if (line == "help") {
        command.kind = Kind::help;
    } else if (line == "status") {
        command.kind = Kind::status;
    } else {
        const auto separator = line.find_first_of(" \t");
        const auto name = line.substr(0U, separator);
        std::string_view payload;
        if (name == "nrfsend" && separator != std::string_view::npos) {
            payload = trim(line.substr(separator));
        } else if (name == "R" && separator != std::string_view::npos) {
            payload = line;
        }
        if (payload.size() > command.payload.size()) {
            command.kind = Kind::too_long;
        } else if (!payload.empty()) {
            command.kind = Kind::transmit;
            std::copy(payload.begin(), payload.end(), command.payload.begin());
        }
    }
    enqueue(command);
    length_ = 0U;
    invalid_ = false;
    too_long_ = false;
}

void SerialCommandQueue::discardPartial() noexcept
{
    // Reject the entire affected line, including its tail after a DMA overrun.
    length_ = 0U;
    invalid_ = true;
}

void SerialCommandQueue::enqueue(const Command& command) noexcept
{
    if (count_ == commands_.size()) {
        ++dropped_;
        return;
    }
    commands_[tail_] = command;
    tail_ = (tail_ + 1U) % commands_.size();
    ++count_;
}

bool SerialCommandQueue::pop(Command& command) noexcept
{
    if (count_ == 0U) {
        return false;
    }
    command = commands_[head_];
    head_ = (head_ + 1U) % commands_.size();
    --count_;
    return true;
}
