#include "RemoteDisplay.hpp"

#include <algorithm>
#include <cstdio>

#include "main.h"
#include "RemoteControlState.hpp"

namespace {
constexpr float kSpeedMinimum = RemoteControlLimits::speed_minimum;
constexpr float kSpeedMaximum = RemoteControlLimits::speed_maximum;
constexpr float kTurnMinimum = RemoteControlLimits::turn_minimum;
constexpr float kTurnMaximum = RemoteControlLimits::turn_maximum;
constexpr float kLegMinimumMm = RemoteControlLimits::leg_minimum_mm;
constexpr float kLegMaximumMm = RemoteControlLimits::leg_maximum_mm;
constexpr float kRollMinimumDegrees = RemoteControlLimits::roll_minimum_degrees;
constexpr float kRollMaximumDegrees = RemoteControlLimits::roll_maximum_degrees;

float clamp(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

std::uint8_t mapToBar(float value, float minimum, float maximum, std::uint8_t pixels)
{
    const float normalized = (clamp(value, minimum, maximum) - minimum) / (maximum - minimum);
    return static_cast<std::uint8_t>(normalized * static_cast<float>(pixels) + 0.5F);
}
} // namespace

RemoteDisplay::RemoteDisplay()
    : oled_({GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9, 0x3C, 0xCF})
{
}

bool RemoteDisplay::initialize()
{
    return oled_.initialize();
}

void RemoteDisplay::render(const RemoteDisplayState& state)
{
    if (!oled_.isInitialized()) {
        return;
    }

    const float speed = clamp(state.speed, kSpeedMinimum, kSpeedMaximum);
    const float turn = clamp(state.turn, kTurnMinimum, kTurnMaximum);
    const float leg = clamp(state.leg_height_mm, kLegMinimumMm, kLegMaximumMm);
    const float roll = clamp(state.roll_degrees, kRollMinimumDegrees, kRollMaximumDegrees);

    char value[16]{};
    auto& u8g2 = oled_.context();
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFontMode(&u8g2, 1);

    // High-contrast title bar.
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 128, 11);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 3, 8, "WL1 REMOTE");
    u8g2_DrawStr(&u8g2, 112, 8, "UI");

    // Speed is the most important live command, so it gets the largest type.
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 2, 18, "SPD");
    std::snprintf(value, sizeof(value), "%+.0f", static_cast<double>(speed));
    u8g2_SetFont(&u8g2, u8g2_font_helvB12_tf);
    u8g2_DrawStr(&u8g2, 2, 32, value);

    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 73, 18, "TURN");
    std::snprintf(value, sizeof(value), "%+.0f", static_cast<double>(turn));
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 73, 31, value);
    u8g2_DrawHLine(&u8g2, 0, 34, 128);

    // Leg-height command and its safe 44.5..78.5 mm range.
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 2, 44, state.leg_locked ? "LEG LOCK" : "LEG LIVE");
    std::snprintf(value, sizeof(value), "%4.1f", static_cast<double>(leg));
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 45, 46, value);
    constexpr std::uint8_t leg_bar_x = 80;
    constexpr std::uint8_t leg_bar_width = 47;
    u8g2_DrawFrame(&u8g2, leg_bar_x, 38, leg_bar_width, 8);
    const auto leg_fill = mapToBar(leg, kLegMinimumMm, kLegMaximumMm, leg_bar_width - 2U);
    if (leg_fill > 0U) {
        u8g2_DrawBox(&u8g2, leg_bar_x + 1U, 39, leg_fill, 6);
    }
    u8g2_DrawHLine(&u8g2, 0, 48, 128);

    // Roll command uses a centered gauge: left/right of center matches sign.
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
    u8g2_DrawStr(&u8g2, 2, 59, state.roll_locked ? "ROLL LOCK" : "ROLL LIVE");
    std::snprintf(value, sizeof(value), "%+5.1f", static_cast<double>(roll));
    u8g2_DrawStr(&u8g2, 48, 59, value);
    constexpr std::uint8_t roll_bar_x = 80;
    constexpr std::uint8_t roll_bar_width = 47;
    u8g2_DrawFrame(&u8g2, roll_bar_x, 51, roll_bar_width, 10);
    u8g2_DrawVLine(&u8g2, roll_bar_x + roll_bar_width / 2U, 52, 8);
    const auto roll_marker = mapToBar(
        roll, kRollMinimumDegrees, kRollMaximumDegrees, roll_bar_width - 5U);
    u8g2_DrawBox(&u8g2, roll_bar_x + 1U + roll_marker, 53, 3, 6);

    oled_.present();
}
