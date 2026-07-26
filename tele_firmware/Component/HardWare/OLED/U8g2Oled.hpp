#pragma once

#include <cstdint>

#include "stm32f4xx_hal.h"

extern "C" {
#include "u8g2.h"
}

struct U8g2OledConfig {
    GPIO_TypeDef* scl_port;
    std::uint16_t scl_pin;
    GPIO_TypeDef* sda_port;
    std::uint16_t sda_pin;
    std::uint8_t i2c_address = 0x3C;
    std::uint8_t contrast = 0xCF;
};

/**
 * SSD1306 128x64 U8G2 adapter using the board's existing software-I2C pins.
 * The GPIO clocks must already be enabled by MX_GPIO_Init().
 */
class U8g2Oled {
public:
    explicit U8g2Oled(U8g2OledConfig config);

    bool initialize();
    bool isInitialized() const { return initialized_; }
    void present();

    u8g2_t& context() { return u8g2_; }

private:
    static std::uint8_t gpioDelayCallback(u8x8_t* u8x8,
                                          std::uint8_t message,
                                          std::uint8_t value,
                                          void* data);
    std::uint8_t onGpioDelay(std::uint8_t message, std::uint8_t value);
    void configureOpenDrainPin(GPIO_TypeDef* port, std::uint16_t pin) const;
    static void delayMicroseconds(std::uint32_t microseconds);

    U8g2OledConfig config_;
    u8g2_t u8g2_{};
    bool initialized_ = false;
};
