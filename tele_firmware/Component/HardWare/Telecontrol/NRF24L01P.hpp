#pragma once

#include <array>
#include <cstdint>

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

class NRF24L01P final {
public:
    static constexpr std::uint8_t packet_width = 32U;

    struct Status {
        bool tx_fifo_full = false;
        std::uint8_t rx_pipe = 7U;
        bool max_retries = false;
        bool tx_sent = false;
        bool rx_ready = false;
    };

    NRF24L01P(
        SPI_HandleTypeDef* spi,
        GPIO_TypeDef* chip_select_port,
        std::uint16_t chip_select_pin,
        GPIO_TypeDef* chip_enable_port,
        std::uint16_t chip_enable_pin,
        GPIO_TypeDef* irq_port,
        std::uint16_t irq_pin) noexcept;

    [[nodiscard]] bool initialize();
    [[nodiscard]] bool startReceive();
    [[nodiscard]] bool send(const std::uint8_t* data, std::uint8_t length);
    [[nodiscard]] bool receive(std::uint8_t* data, std::uint8_t length = packet_width);
    [[nodiscard]] bool handleIrq(Status& status);

    [[nodiscard]] SPI_HandleTypeDef* spiHandle() const noexcept { return spi_; }
    [[nodiscard]] std::uint16_t irqPin() const noexcept { return irq_pin_; }

    void onSpiTransferCompleteFromIsr() noexcept;

private:
    static constexpr std::uint8_t command_read_register = 0x00U;
    static constexpr std::uint8_t command_write_register = 0x20U;
    static constexpr std::uint8_t command_read_rx_payload = 0x61U;
    static constexpr std::uint8_t command_write_tx_payload = 0xA0U;
    static constexpr std::uint8_t command_flush_tx = 0xE1U;
    static constexpr std::uint8_t command_flush_rx = 0xE2U;

    static constexpr std::uint8_t register_config = 0x00U;
    static constexpr std::uint8_t register_enable_auto_ack = 0x01U;
    static constexpr std::uint8_t register_enable_rx_address = 0x02U;
    static constexpr std::uint8_t register_address_width = 0x03U;
    static constexpr std::uint8_t register_retransmit = 0x04U;
    static constexpr std::uint8_t register_rf_channel = 0x05U;
    static constexpr std::uint8_t register_rf_setup = 0x06U;
    static constexpr std::uint8_t register_status = 0x07U;
    static constexpr std::uint8_t register_rx_address_pipe0 = 0x0AU;
    static constexpr std::uint8_t register_tx_address = 0x10U;
    static constexpr std::uint8_t register_rx_width_pipe0 = 0x11U;

    static constexpr TickType_t spi_timeout = pdMS_TO_TICKS(10U);
    static constexpr std::array<std::uint8_t, 5U> radio_address{
        0x11U, 0x52U, 0x01U, 0x31U, 0x41U};

    [[nodiscard]] bool ensureSemaphore();
    [[nodiscard]] bool spiSend(const std::uint8_t* data, std::uint16_t size);
    [[nodiscard]] bool spiReceive(std::uint8_t* data, std::uint16_t size);
    [[nodiscard]] bool readRegister(
        std::uint8_t reg, std::uint8_t* data, std::uint8_t length);
    [[nodiscard]] bool writeRegister(
        std::uint8_t reg, const std::uint8_t* data, std::uint8_t length);
    [[nodiscard]] bool command(std::uint8_t value);
    [[nodiscard]] bool flushTx();
    [[nodiscard]] bool flushRx();
    [[nodiscard]] bool startTransmit();
    [[nodiscard]] bool readStatus(Status& status);
    [[nodiscard]] bool clearStatus(std::uint8_t flags = 0x70U);

    void setChipSelect(bool active) noexcept;
    void setChipEnable(bool enabled) noexcept;

    SPI_HandleTypeDef* spi_;
    GPIO_TypeDef* chip_select_port_;
    std::uint16_t chip_select_pin_;
    GPIO_TypeDef* chip_enable_port_;
    std::uint16_t chip_enable_pin_;
    GPIO_TypeDef* irq_port_;
    std::uint16_t irq_pin_;
    StaticSemaphore_t spi_semaphore_storage_{};
    SemaphoreHandle_t spi_semaphore_ = nullptr;
};
