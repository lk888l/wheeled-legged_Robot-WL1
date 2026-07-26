#include "U8g2Oled.hpp"

U8g2Oled::U8g2Oled(U8g2OledConfig config)
    : config_(config)
{
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2_, U8G2_R0, u8x8_byte_sw_i2c, gpioDelayCallback);
    u8g2_SetUserPtr(&u8g2_, this);
    u8g2_SetI2CAddress(&u8g2_, static_cast<std::uint8_t>(config_.i2c_address << 1U));
}

bool U8g2Oled::initialize()
{
    initialized_ = false;
    u8g2_InitDisplay(&u8g2_);
    u8g2_SetPowerSave(&u8g2_, 0);
    u8g2_SetContrast(&u8g2_, config_.contrast);
    u8g2_ClearBuffer(&u8g2_);
    u8g2_SendBuffer(&u8g2_);
    initialized_ = true;
    return true;
}

void U8g2Oled::present()
{
    if (initialized_) {
        u8g2_SendBuffer(&u8g2_);
    }
}

std::uint8_t U8g2Oled::gpioDelayCallback(u8x8_t* u8x8,
                                         std::uint8_t message,
                                         std::uint8_t value,
                                         void*)
{
    auto* self = static_cast<U8g2Oled*>(u8x8_GetUserPtr(u8x8));
    return self == nullptr ? 0U : self->onGpioDelay(message, value);
}

std::uint8_t U8g2Oled::onGpioDelay(std::uint8_t message, std::uint8_t value)
{
    switch (message) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        configureOpenDrainPin(config_.scl_port, config_.scl_pin);
        configureOpenDrainPin(config_.sda_port, config_.sda_pin);
        return 1U;
    case U8X8_MSG_GPIO_I2C_CLOCK:
        HAL_GPIO_WritePin(config_.scl_port, config_.scl_pin,
                          value != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return 1U;
    case U8X8_MSG_GPIO_I2C_DATA:
        HAL_GPIO_WritePin(config_.sda_port, config_.sda_pin,
                          value != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return 1U;
    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(value);
        return 1U;
    case U8X8_MSG_DELAY_10MICRO:
        delayMicroseconds(static_cast<std::uint32_t>(value) * 10U);
        return 1U;
    case U8X8_MSG_DELAY_100NANO:
        if (value != 0U) {
            delayMicroseconds((static_cast<std::uint32_t>(value) + 9U) / 10U);
        }
        return 1U;
    case U8X8_MSG_DELAY_NANO:
        if (value != 0U) {
            delayMicroseconds((static_cast<std::uint32_t>(value) + 999U) / 1000U);
        }
        return 1U;
    case U8X8_MSG_DELAY_I2C:
        // U8G2 passes the bus speed in units of 100 kHz (4 for this SSD1306).
        delayMicroseconds(value <= 2U ? 5U : 1U);
        return 1U;
    case U8X8_MSG_GPIO_RESET:
        // The four-pin OLED module has no separate reset signal.
        return 1U;
    default:
        return 1U;
    }
}

void U8g2Oled::configureOpenDrainPin(GPIO_TypeDef* port, std::uint16_t pin) const
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio{};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

void U8g2Oled::delayMicroseconds(std::uint32_t microseconds)
{
    // The inline NOP loop also works before the RTOS scheduler starts.
    const std::uint32_t loops_per_microsecond =
        (SystemCoreClock / 3'000'000U) > 0U ? (SystemCoreClock / 3'000'000U) : 1U;
    while (microseconds-- > 0U) {
        for (std::uint32_t loop = 0; loop < loops_per_microsecond; ++loop) {
            __NOP();
        }
    }
}
