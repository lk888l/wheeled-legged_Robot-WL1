#include "LkUart.hpp"

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    LkUart<>::isrTxComplete(uart);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size)
{
    LkUart<>::isRxComplete(uart, size);
}
