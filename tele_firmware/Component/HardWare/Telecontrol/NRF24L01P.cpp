#include "NRF24L01P.hpp"

#include <algorithm>

NRF24L01P::NRF24L01P(
    SPI_HandleTypeDef* spi,
    GPIO_TypeDef* chip_select_port,
    std::uint16_t chip_select_pin,
    GPIO_TypeDef* chip_enable_port,
    std::uint16_t chip_enable_pin,
    GPIO_TypeDef* irq_port,
    std::uint16_t irq_pin) noexcept
    : spi_(spi),
      chip_select_port_(chip_select_port),
      chip_select_pin_(chip_select_pin),
      chip_enable_port_(chip_enable_port),
      chip_enable_pin_(chip_enable_pin),
      irq_port_(irq_port),
      irq_pin_(irq_pin)
{
}

bool NRF24L01P::ensureSemaphore()
{
    if (spi_semaphore_ == nullptr) {
        spi_semaphore_ = xSemaphoreCreateBinaryStatic(&spi_semaphore_storage_);
    }
    if (spi_semaphore_ == nullptr) {
        return false;
    }

    static_cast<void>(xSemaphoreTake(spi_semaphore_, 0U));
    return true;
}

void NRF24L01P::setChipSelect(bool active) noexcept
{
    HAL_GPIO_WritePin(
        chip_select_port_, chip_select_pin_, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void NRF24L01P::setChipEnable(bool enabled) noexcept
{
    HAL_GPIO_WritePin(
        chip_enable_port_, chip_enable_pin_, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool NRF24L01P::spiSend(const std::uint8_t* data, std::uint16_t size)
{
    if (spi_semaphore_ == nullptr || data == nullptr || size == 0U) {
        return false;
    }

    static_cast<void>(xSemaphoreTake(spi_semaphore_, 0U));
    if (HAL_SPI_Transmit_DMA(spi_, const_cast<std::uint8_t*>(data), size) != HAL_OK) {
        return false;
    }
    return xSemaphoreTake(spi_semaphore_, spi_timeout) == pdTRUE &&
           HAL_SPI_GetError(spi_) == HAL_SPI_ERROR_NONE;
}

bool NRF24L01P::spiReceive(std::uint8_t* data, std::uint16_t size)
{
    if (spi_semaphore_ == nullptr || data == nullptr || size == 0U) {
        return false;
    }

    static_cast<void>(xSemaphoreTake(spi_semaphore_, 0U));
    if (HAL_SPI_Receive_DMA(spi_, data, size) != HAL_OK) {
        return false;
    }
    return xSemaphoreTake(spi_semaphore_, spi_timeout) == pdTRUE &&
           HAL_SPI_GetError(spi_) == HAL_SPI_ERROR_NONE;
}

bool NRF24L01P::readRegister(
    std::uint8_t reg, std::uint8_t* data, std::uint8_t length)
{
    setChipSelect(true);
    const std::uint8_t address = command_read_register | reg;
    const bool success = spiSend(&address, 1U) && spiReceive(data, length);
    setChipSelect(false);
    return success;
}

bool NRF24L01P::writeRegister(
    std::uint8_t reg, const std::uint8_t* data, std::uint8_t length)
{
    setChipSelect(true);
    const std::uint8_t address = command_write_register | reg;
    const bool success = spiSend(&address, 1U) && spiSend(data, length);
    setChipSelect(false);
    return success;
}

bool NRF24L01P::command(std::uint8_t value)
{
    setChipSelect(true);
    const bool success = spiSend(&value, 1U);
    setChipSelect(false);
    return success;
}

bool NRF24L01P::flushTx()
{
    return command(command_flush_tx);
}

bool NRF24L01P::flushRx()
{
    return command(command_flush_rx);
}

bool NRF24L01P::initialize()
{
    if (!ensureSemaphore()) {
        return false;
    }

    setChipEnable(false);
    setChipSelect(false);

    bool success = true;
    std::uint8_t value = 0x08U; // CRC enabled, powered down, PTX.
    success = writeRegister(register_config, &value, 1U) && success;

    value = 0x3FU;
    success = writeRegister(register_enable_auto_ack, &value, 1U) && success;
    value = 0x03U;
    success = writeRegister(register_enable_rx_address, &value, 1U) && success;
    value = 0x03U; // Five-byte addresses.
    success = writeRegister(register_address_width, &value, 1U) && success;
    value = 0x1FU; // 500 us retry delay, 15 retries.
    success = writeRegister(register_retransmit, &value, 1U) && success;
    value = 2U;
    success = writeRegister(register_rf_channel, &value, 1U) && success;

    std::uint8_t rf_setup = 0U;
    success = readRegister(register_rf_setup, &rf_setup, 1U) && success;
    rf_setup = static_cast<std::uint8_t>((rf_setup & ~0x28U) | 0x08U); // 2 Mbps.
    rf_setup = static_cast<std::uint8_t>((rf_setup & ~0x06U) | 0x06U); // 0 dBm.
    success = writeRegister(register_rf_setup, &rf_setup, 1U) && success;

    success = writeRegister(
                  register_rx_address_pipe0,
                  radio_address.data(),
                  static_cast<std::uint8_t>(radio_address.size())) &&
              success;
    success = writeRegister(
                  register_tx_address,
                  radio_address.data(),
                  static_cast<std::uint8_t>(radio_address.size())) &&
              success;
    value = packet_width;
    success = writeRegister(register_rx_width_pipe0, &value, 1U) && success;

    success = flushRx() && success;
    success = flushTx() && success;
    success = clearStatus() && success;
    return success;
}

bool NRF24L01P::startReceive()
{
    setChipEnable(false);

    std::uint8_t config = 0U;
    if (!readRegister(register_config, &config, 1U)) {
        return false;
    }
    config = static_cast<std::uint8_t>(config | 0x03U);
    if (!writeRegister(register_config, &config, 1U)) {
        return false;
    }

    setChipEnable(true);
    return true;
}

bool NRF24L01P::startTransmit()
{
    setChipEnable(false);

    std::uint8_t config = 0U;
    if (!readRegister(register_config, &config, 1U)) {
        return false;
    }
    config = static_cast<std::uint8_t>((config | 0x02U) & ~0x01U);
    if (!writeRegister(register_config, &config, 1U)) {
        return false;
    }

    setChipEnable(true);
    return true;
}

bool NRF24L01P::send(const std::uint8_t* data, std::uint8_t length)
{
    if (data == nullptr || length == 0U) {
        return false;
    }
    length = std::min(length, packet_width);

    setChipEnable(false);
    setChipSelect(true);
    const std::uint8_t command_value = command_write_tx_payload;
    const bool payload_written =
        spiSend(&command_value, 1U) && spiSend(data, length);
    setChipSelect(false);
    return payload_written && startTransmit();
}

bool NRF24L01P::receive(std::uint8_t* data, std::uint8_t length)
{
    if (data == nullptr || length == 0U) {
        return false;
    }
    length = std::min(length, packet_width);

    setChipSelect(true);
    const std::uint8_t command_value = command_read_rx_payload;
    const bool success =
        spiSend(&command_value, 1U) && spiReceive(data, length);
    setChipSelect(false);
    return success && clearStatus(0x40U);
}

bool NRF24L01P::hasReceivedPayload(bool& available)
{
    std::uint8_t fifo_status = 0U;
    if (!readRegister(register_fifo_status, &fifo_status, 1U)) {
        return false;
    }
    available = (fifo_status & 0x01U) == 0U;
    return true;
}

bool NRF24L01P::readStatus(Status& status)
{
    std::uint8_t value = 0U;
    if (!readRegister(register_status, &value, 1U)) {
        return false;
    }

    status.tx_fifo_full = (value & 0x01U) != 0U;
    status.rx_pipe = static_cast<std::uint8_t>((value >> 1U) & 0x07U);
    status.max_retries = (value & 0x10U) != 0U;
    status.tx_sent = (value & 0x20U) != 0U;
    status.rx_ready = (value & 0x40U) != 0U;
    return true;
}

bool NRF24L01P::clearStatus(std::uint8_t flags)
{
    return flags == 0U || writeRegister(register_status, &flags, 1U);
}

bool NRF24L01P::handleIrq(Status& status)
{
    if (!readStatus(status)) {
        return false;
    }

    // Leave RX data and RX_DR for receive(); never discard a car's reply here.
    const std::uint8_t flags = (status.tx_sent ? 0x20U : 0U) |
                               (status.max_retries ? 0x10U : 0U);
    bool success = clearStatus(flags);
    if (status.max_retries) {
        success = flushTx() && success;
    }
    if (status.tx_sent || status.max_retries) {
        success = startReceive() && success;
    }
    return success;
}

void NRF24L01P::onSpiTransferCompleteFromIsr() noexcept
{
    if (spi_semaphore_ == nullptr) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(spi_semaphore_, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
