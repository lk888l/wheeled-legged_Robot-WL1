#pragma once

// ZX-D30 factory UART is 9600 8N1. Transparent UART: no AT handshake or remote connection is required to boot.
// Override from CMake when the module has already been configured differently.
#ifndef WL1_COMMAND_UART_BAUD
#define WL1_COMMAND_UART_BAUD 9600U
#endif

// The Bluetooth build does not probe or listen to the SPI remote by default.
#ifndef WL1_ENABLE_NRF24
#define WL1_ENABLE_NRF24 0
#endif
