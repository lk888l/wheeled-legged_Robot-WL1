#pragma once
#include <cstdint>
struct TIM_HandleTypeDef { uint32_t compare{}; bool enabled{}; };
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* timer, uint32_t) {
    timer->enabled = true; return HAL_OK;
}
inline HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* timer, uint32_t) {
    timer->enabled = false; return HAL_OK;
}
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((timer)->compare = (value))
