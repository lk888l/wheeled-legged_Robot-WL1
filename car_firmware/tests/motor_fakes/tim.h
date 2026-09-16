#pragma once
#include <array>
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
struct GPIO_TypeDef { uint16_t pins{}; };
struct TIM_TypeDef { uint32_t counter{}; std::array<uint32_t, 2> compare{}; };
inline TIM_TypeDef tim2_instance, tim3_instance;
inline auto* TIM2 = &tim2_instance;
inline auto* TIM3 = &tim3_instance;
struct TIM_HandleTypeDef { TIM_TypeDef* Instance{}; };
inline constexpr uint32_t TIM_CHANNEL_1 = 0, TIM_CHANNEL_2 = 1, TIM_CHANNEL_ALL = 0xffff;
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->Instance->counter)
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->Instance->counter = (value))
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->Instance->compare.at(channel) = (value))
inline HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline void HAL_GPIO_WritePin(GPIO_TypeDef* gpio, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) gpio->pins |= pin;
    else gpio->pins &= ~pin;
}
