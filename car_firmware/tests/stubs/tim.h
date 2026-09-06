#pragma once

#include <cstdint>

struct TIM_TypeDef { std::uint32_t CCR[4]{}; };
struct TIM_HandleTypeDef { TIM_TypeDef* Instance; };
struct GPIO_TypeDef { std::uint32_t ODR{}; };
enum GPIO_PinState { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 };
inline constexpr std::uint32_t TIM_CHANNEL_1 = 0;
inline constexpr std::uint32_t TIM_CHANNEL_2 = 4;
inline constexpr std::uint8_t HAL_OK = 0;

inline std::uint8_t HAL_TIM_PWM_Start(TIM_HandleTypeDef*, std::uint32_t) { return HAL_OK; }
inline std::uint8_t HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, std::uint32_t) { return HAL_OK; }
inline void HAL_GPIO_WritePin(GPIO_TypeDef* gpio, std::uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) gpio->ODR |= pin;
    else gpio->ODR &= ~static_cast<std::uint32_t>(pin);
}
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->Instance->CCR[(channel)/4] = (value))
