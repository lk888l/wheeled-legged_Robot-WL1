#include <cstring>
#include <string>
#include <vector>
#include "../Component/Peripheral/LkUart.hpp"
#include "test_check.hpp"

int main()
{
    fake_rtos::reset();
    DMA_HandleTypeDef dma;
    UART_HandleTypeDef hal{&dma};
    using Uart = LkUart<128U, 2U, 128U, 3U>;
    Uart uart(&hal);
    hal.fail_rx = true;
    CHECK(!uart.Start_DMAIT_Receive());
    hal.fail_rx = false;
    CHECK(uart.Start_DMAIT_Receive());
    CHECK((dma.interrupts & DMA_IT_HT) == 0U);
    CHECK(uart.Start_DMAIT_Receive() && hal.rx_starts == 1U);
    CHECK(uart.bindReactor(&Uart::signal_RxComplete, xTaskGetCurrentTaskHandle(), 1U));
    std::vector<std::string> received;
    const auto drain = [&] {
        uart.signal_RxComplete([&](const etl::string<128U>& s) { received.emplace_back(s.data(), s.size()); });
    };
    const auto deliver = [&](const std::string& text, HAL_UART_RxEventTypeTypeDef event = HAL_UART_RXEVENT_IDLE) {
        CHECK(text.size() <= hal.rx_capacity);
        std::memcpy(hal.rx_buffer, text.data(), text.size());
        hal.event = event;
        if (event != HAL_UART_RXEVENT_HT) { hal.RxState = HAL_UART_STATE_READY; }
        Uart::isRxComplete(&hal, static_cast<uint16_t>(text.size()));
    };
    deliver(std::string(64U, 'x'), HAL_UART_RXEVENT_HT);
    drain();
    CHECK(received.empty() && hal.rx_starts == 1U);
    deliver(std::string(128U, 'x'), HAL_UART_RXEVENT_TC);
    drain();
    CHECK(received.size() == 1U && received[0].size() == 128U);
    CHECK((dma.interrupts & DMA_IT_HT) == 0U);
    received.clear();
    deliver("first");
    deliver("second");
    deliver("overflow");
    drain();
    CHECK((received == std::vector<std::string>{"first", "second"}));
    deliver("after-gap");
    drain();
    CHECK(received.size() == 4U && received[2].empty() && received[3] == "after-gap");
    received.clear();
    hal.RxState = HAL_UART_STATE_READY; // HAL has aborted RX on ORE/FE/DMA error.
    Uart::isrError(&hal);
    CHECK(hal.RxState == HAL_UART_STATE_BUSY_RX);
    deliver("@ping\n");
    drain();
    CHECK((received == std::vector<std::string>{"", "@ping\n"}));
    received.clear();
    hal.fail_rx = true;
    deliver("before-rearm-failure");
    CHECK(hal.RxState == HAL_UART_STATE_READY);
    hal.fail_rx = false;
    uart.service_receive();
    CHECK(hal.RxState == HAL_UART_STATE_BUSY_RX);
    drain();
    CHECK(received.back() == "before-rearm-failure");
    // A rejected DMA transmit must release its buffer and busy flag.
    hal.fail_tx = true;
    for (unsigned i = 0; i < 8U; ++i) { uart.print("test {}", i); }
    hal.fail_tx = false;
    uart.print("test");
    CHECK(hal.tx_starts == 1U);
    Uart::isrTxComplete(&hal);
    uart.print("again");
    CHECK(hal.tx_starts == 2U);
    CHECK(fake_rtos::critical_depth == 0);
}
