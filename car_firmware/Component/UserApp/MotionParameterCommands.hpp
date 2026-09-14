#pragma once

#include "MotionParameters.hpp"
#include <charconv>
#include <string_view>

namespace MotionSettings {

inline std::string_view trim(std::string_view text) noexcept
{
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return {};
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

inline std::string_view takeToken(std::string_view& text) noexcept
{
    text = trim(text);
    const auto end = text.find_first_of(" \t\r\n");
    const auto token = text.substr(0, end);
    text = end == std::string_view::npos ? std::string_view{} : trim(text.substr(end));
    return token;
}

inline bool parseFloat(std::string_view& args, float& value) noexcept
{
    const auto token = takeToken(args);
    if (token.empty()) return false;
    const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
    return result.ec == std::errc{} && result.ptr == token.data() + token.size() &&
        BalanceCompensation::isFiniteBias(value);
}

inline bool parseDeadzone(std::string_view& args, std::uint16_t& value) noexcept
{
    const auto token = takeToken(args);
    if (token.empty()) return false;
    const auto result = std::from_chars(token.data(), token.data() + token.size(), value);
    return result.ec == std::errc{} && result.ptr == token.data() + token.size() &&
        value <= maximum_motor_deadzone;
}

// Apply to a copy so malformed/truncated packets never partially update a command.
inline bool applyTuning(Parameters& p, std::string_view name, std::string_view args) noexcept
{
    if (name == "deadzone") {
        std::uint16_t value{};
        if (!parseDeadzone(args, value) || !args.empty()) return false;
        p.motor_deadzone = value;
        return true;
    }
    PidGains* pid = name == "anglepid" ? &p.angle : name == "velocitypid" ? &p.velocity :
        name == "differpid" ? &p.difference :
        (name == "rollpid" || name == "legpid") ? &p.roll : nullptr;
    float value{};
    if (pid) {
        const auto option = takeToken(args);
        if (option != "-p" && option != "-i" && option != "-d") return false;
        if (!parseFloat(args, value) || !args.empty()) return false;
        if (option == "-p") pid->kp = value;
        else if (option == "-i") pid->ki = value;
        else pid->kd = value;
        return true;
    }
    if (name != "anglebias" && name != "legheight" && name != "target_roll") return false;
    if (!parseFloat(args, value) || !args.empty()) return false;
    if (name == "anglebias") p.minimum_pitch_bias = value;
    else if (name == "legheight") p.leg_height = BalanceCompensation::clampLegHeight(value);
    else p.roll_target = value;
    return true;
}

enum class SaveMode { invalid, append, recycle };

inline SaveMode parseSaveMode(std::string_view args) noexcept
{
    args = trim(args);
    if (args.empty() || args == "all") return SaveMode::append;
    if (args == "recycle") return SaveMode::recycle;
    return SaveMode::invalid;
}

} // namespace MotionSettings
