/********************************************************************************
  * @file           : LkUart.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-4
  *******************************************************************************/
#include "LkUart.hpp"

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    LkUart<>::isrTxComplete(huart);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    LkUart<>::isRxComplete(huart, size);
}

