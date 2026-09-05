#pragma once
#include <cstdint>
inline constexpr int HAL_OK = 0;
inline constexpr int HAL_BUSY = 1;
inline constexpr int HAL_UART_STATE_READY = 0;
inline constexpr int HAL_UART_STATE_BUSY_RX = 1;
inline constexpr std::uint32_t HAL_UART_ERROR_NONE = 0U;
struct UART_HandleTypeDef {
    int RxState = HAL_UART_STATE_READY;
    std::uint32_t ErrorCode = HAL_UART_ERROR_NONE;
    std::uint8_t* receiving = nullptr;
};
inline int HAL_UART_Receive_IT(UART_HandleTypeDef* uart, std::uint8_t* byte, int)
{
    if (uart->RxState != HAL_UART_STATE_READY) { return HAL_BUSY; }
    uart->receiving = byte;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    return HAL_OK;
}
