#pragma once
#include <cstdint>
#include "FreeRTOS.h"

#define taskENTER_CRITICAL_FROM_ISR() (uint32_t{0})
#define taskEXIT_CRITICAL_FROM_ISR(mask) ((void)(mask))
#define portYIELD_FROM_ISR(woken) ((void)(woken))

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
enum HAL_UART_RxEventTypeTypeDef { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE };
inline constexpr unsigned HAL_UART_STATE_READY = 0x20U;
inline constexpr unsigned HAL_UART_STATE_BUSY_RX = 0x22U;
inline constexpr uint32_t DMA_IT_HT = 0x10U;
struct DMA_HandleTypeDef { uint32_t interrupts{}; };
struct UART_HandleTypeDef {
    DMA_HandleTypeDef* hdmarx{};
    unsigned RxState{HAL_UART_STATE_READY};
    HAL_UART_RxEventTypeTypeDef event{HAL_UART_RXEVENT_IDLE};
    uint8_t* rx_buffer{};
    uint16_t rx_capacity{};
    unsigned rx_starts{}, tx_starts{};
    bool fail_rx{}, fail_tx{};
};
#define __HAL_DMA_DISABLE_IT(dma, flag) ((dma)->interrupts &= ~(flag))
inline HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef* uart) { return uart->event; }
inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* uart, uint8_t* data, uint16_t size)
{
    if (uart->fail_rx) { return HAL_ERROR; }
    if (uart->RxState != HAL_UART_STATE_READY) { return HAL_BUSY; }
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    uart->rx_buffer = data;
    uart->rx_capacity = size;
    uart->hdmarx->interrupts |= DMA_IT_HT;
    ++uart->rx_starts;
    return HAL_OK;
}
inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef* uart, uint8_t*, uint16_t)
{
    if (uart->fail_tx) { return HAL_ERROR; }
    ++uart->tx_starts;
    return HAL_OK;
}
