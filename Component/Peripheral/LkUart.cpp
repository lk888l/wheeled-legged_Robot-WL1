/********************************************************************************
  * @file           : LkUart.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-4
  *******************************************************************************/


#include "LkUart.hpp"

LkUart::LkUart(UART_HandleTypeDef *_huart)
    :HUart(_huart)
{

}

void LkUart::UART_SendString(const char *str, size_t len) {
    HAL_UART_Transmit(HUart, (uint8_t*)str, len, HAL_MAX_DELAY);
}


