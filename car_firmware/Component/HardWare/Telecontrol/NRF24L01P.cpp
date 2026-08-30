/********************************************************************************
  * @file           : NRF24L01P.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-27
  *******************************************************************************/


#include "NRF24L01P.hpp"
// cpp library include
#include <algorithm>
//freeRTOS library include
#include "semphr.h"

NRF24L01P::NRF24L01P(SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *csPort, uint16_t csPin,
                     GPIO_TypeDef *cePort,uint16_t cePin,
                     GPIO_TypeDef* irqPort, uint16_t irqPin)
    : HSpi(hspi), csPort(csPort), csPin(csPin), cePort(cePort), cePin(cePin),irqPort(irqPort), irqPin(irqPin)
{
    // GPIO is initialized before the C++ runtime is created. Restore the nRF's
    // inactive bus state immediately and keep it safe across generated-code changes.
    write_ce(0);
    csHigh();
}

/**
 * @brief
 * @return
 */
SPI_HandleTypeDef *NRF24L01P::getSPIHandle(){
    return HSpi;
}

uint16_t NRF24L01P::getIRQGPIOPort(){
    return irqPin;
}

bool NRF24L01P::isIRQAsserted() const {
    return HAL_GPIO_ReadPin(irqPort, irqPin) == GPIO_PIN_RESET;
}

void NRF24L01P::csLow() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
}

void NRF24L01P::csHigh() {
    HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
}

void NRF24L01P::write_ce(uint8_t gpio_status) {
    HAL_GPIO_WritePin(cePort, cePin, static_cast<GPIO_PinState>(gpio_status));
}

bool NRF24L01P::ensureSemaphore() {
    if (spiDmaSemaphore == nullptr) {
        spiDmaSemaphore = xSemaphoreCreateBinaryStatic(&spiDmaSemaphoreStorage);
    }
    if (spiDmaSemaphore == nullptr) {
        return false;
    }
    while (xSemaphoreTake(spiDmaSemaphore, 0) == pdTRUE) {
    }
    return true;
}

bool NRF24L01P::spiSend(const uint8_t *pData, uint16_t size) {
    if (pData == nullptr || size == 0 || !ensureSemaphore()) {
        return false;
    }
    if (HAL_SPI_Transmit_DMA(HSpi, const_cast<uint8_t*>(pData), size) != HAL_OK) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (xSemaphoreTake(spiDmaSemaphore, SPI_DMA_TIMEOUT) != pdTRUE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (HAL_SPI_GetState(HSpi) != HAL_SPI_STATE_READY
        || HAL_SPI_GetError(HSpi) != HAL_SPI_ERROR_NONE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    return true;
}

bool NRF24L01P::spiReceive(uint8_t *pData, uint16_t size) {
    if (pData == nullptr || size == 0 || !ensureSemaphore()) {
        return false;
    }
    if (HAL_SPI_Receive_DMA(HSpi, pData, size) != HAL_OK) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (xSemaphoreTake(spiDmaSemaphore, SPI_DMA_TIMEOUT) != pdTRUE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (HAL_SPI_GetState(HSpi) != HAL_SPI_STATE_READY
        || HAL_SPI_GetError(HSpi) != HAL_SPI_ERROR_NONE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    return true;
}

bool NRF24L01P::spiTransfer(const uint8_t *pTxData, uint8_t *pRxData, uint16_t size) {
    if (pTxData == nullptr || pRxData == nullptr || size == 0 || !ensureSemaphore()) {
        return false;
    }
    if (HAL_SPI_TransmitReceive_DMA(
            HSpi, const_cast<uint8_t*>(pTxData), pRxData, size) != HAL_OK) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (xSemaphoreTake(spiDmaSemaphore, SPI_DMA_TIMEOUT) != pdTRUE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    if (HAL_SPI_GetState(HSpi) != HAL_SPI_STATE_READY
        || HAL_SPI_GetError(HSpi) != HAL_SPI_ERROR_NONE) {
        HAL_SPI_Abort(HSpi);
        return false;
    }
    return true;
}

bool NRF24L01P::readRegister(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (buf == nullptr || len == 0 || len > PACKET_WIDTH) {
        return false;
    }
    std::array<uint8_t, PACKET_WIDTH + 1> tx{};
    std::array<uint8_t, PACKET_WIDTH + 1> rx{};
    tx[0] = static_cast<uint8_t>(CMD_R_REGISTER | reg);
    csLow();
    const bool isSuccess = spiTransfer(tx.data(), rx.data(), len + 1U);
    csHigh();
    if (isSuccess) {
        std::copy_n(rx.begin() + 1, len, buf);
    }
    return isSuccess;
}

bool NRF24L01P::writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len){
    if (buf == nullptr || len == 0 || len > PACKET_WIDTH) {
        return false;
    }
    std::array<uint8_t, PACKET_WIDTH + 1> tx{};
    tx[0] = static_cast<uint8_t>(CMD_W_REGISTER | reg);
    std::copy_n(buf, len, tx.begin() + 1);
    csLow();
    const bool isSuccess = spiSend(tx.data(), len + 1U);
    csHigh();
    return isSuccess;
}

bool NRF24L01P::sendCommand(uint8_t command) {
    csLow();
    const bool success = spiSend(&command, 1);
    csHigh();
    return success;
}

bool NRF24L01P::flushTx() {
    return sendCommand(CMD_FLUSH_TX);
}

bool NRF24L01P::flushRx() {
    return sendCommand(CMD_FLUSH_RX);
}

/**
 * @brief
 */
bool NRF24L01P::Init() {
    if (!ensureSemaphore()) {
        return false;
    }
    write_ce(0);
    csHigh();

    bool isSuccess = true;
    uint8_t command;
    // Enable one-byte CRC while remaining powered down in PTX mode. start_RxMode()
    // performs the power-up transition after all registers have been verified.
    command = 0x08;
    isSuccess = writeRegister(REG_CONFIG, &command,1) && isSuccess;
    // Auto ACK and RX are only needed on pipe 0.
    command = 0x01;
    isSuccess &= writeRegister(REG_EN_AA,&command,1);
    command = 0x01;
    isSuccess &= writeRegister(REG_EN_RXADDR,&command,1);
    //Set Address Widths
    command = 0x03;
    isSuccess &= writeRegister(REG_SETUP_AW,&command,1);
    // Set Auto Retransmit Delay to 500us and Up to 3 Re-Transmits [cite: 775]
//    command = (0x00 << 4) | 0x03;
    command = 0x1F;
    isSuccess &= writeRegister(REG_SETUP_RETR, &command, 1);
    isSuccess &= setChannel(2);
//    command = 0x0E;
//    isSuccess &= writeRegister(REG_RF_SETUP,&command,1);
    isSuccess &= setDataRate(2); // Default to 2Mbps
    isSuccess &= setPALevel(3);  // Default to 0dBm

    isSuccess &= setRX_Addr(0,RxAddress_P0);
    isSuccess &= setRX_PW(0,PACKET_WIDTH);
    isSuccess &= setTX_Addr(TxAddress);
    isSuccess = flushRx() && isSuccess;
    isSuccess = flushTx() && isSuccess;
    command = 0x70;
    isSuccess &= writeRegister(REG_STATUS,&command,1);

    // Read back the settings that identify the radio link. A floating MISO line
    // otherwise makes a failed power-up indistinguishable from a configured radio.
    uint8_t readback = 0;
    isSuccess = readRegister(REG_RF_CH, &readback, 1) && readback == 2U && isSuccess;
    isSuccess = readRegister(REG_SETUP_AW, &readback, 1)
        && (readback & 0x03U) == 0x03U && isSuccess;
    isSuccess = readRegister(REG_RX_PW_P0, &readback, 1)
        && readback == PACKET_WIDTH && isSuccess;
    isSuccess = readRegister(REG_RF_SETUP, &readback, 1)
        && (readback & 0x28U) == 0x08U && isSuccess;
    return isSuccess;
}

bool NRF24L01P::setChannel(uint8_t channel) {
    // F0 = 2400 + RF_CH [MHz] [cite: 394]
    if (channel > 125) channel = 125;
    return writeRegister(REG_RF_CH, &channel,1);
}

/**
 * @brief Set RF output power in TX mode: '00' -18dBm, '01' -12dBm, '10' -6dBm, '11' 0dBm [cite: 783]
 * @param level
 * @return
 */
bool NRF24L01P::setPALevel(uint8_t level) {
    uint8_t setup{};
    if (!readRegister(REG_RF_SETUP, &setup, 1)) {
        return false;
    }
    setup &= (~0x06);
    if (level > 3) level = 3;
    setup |= (level << 1);
    return writeRegister(REG_RF_SETUP, &setup,1);
}

/**
 * @brief Encoding: '00' 1Mbps, '01' 2Mbps, '10' 250kbps [cite: 781]
 * @param rate
 * @return
 */
bool NRF24L01P::setDataRate(uint8_t rate) {
    uint8_t setup{};
    if (!readRegister(REG_RF_SETUP, &setup, 1)) {
        return false;
    }
    // Clear RF_DR_LOW and RF_DR_HIGH.
    setup  &= (~0x28);
    if (rate == 0) setup |= (1 << 5);       // 250kbps (RF_DR_LOW = 1)
    else if (rate == 1) setup &= ~(1 << 3); // 1Mbps (RF_DR_HIGH = 0)
    else if (rate == 2) setup |= (1 << 3);  // 2Mbps (RF_DR_HIGH = 1)
    return writeRegister(REG_RF_SETUP, &setup,1);
}


bool NRF24L01P::setRX_Addr(uint8_t pipeNum, uint8_t *address) {
    uint8_t REG_ADDR;
    uint8_t len = 5;
    if(pipeNum > 5) pipeNum = 5;
    if(pipeNum == 0) REG_ADDR = REG_RX_ADDR_P0;
    else if(pipeNum == 1) REG_ADDR = REG_RX_ADDR_P1;
    else {
        REG_ADDR = REG_RX_ADDR_P2 + (pipeNum - 2);
        len = 1; // P2-P5 只有 1 字节
    }
    return writeRegister(REG_ADDR, address, len);
}

bool NRF24L01P::setRX_PW(uint8_t pipeNum, uint8_t size) {
    uint8_t REG_ADDR;
    if(pipeNum>5) pipeNum=5;
    if(size>32) size = 32;
    if(pipeNum == 0)  REG_ADDR=  REG_RX_PW_P0;
    else if(pipeNum == 1)  REG_ADDR = REG_RX_PW_P1;
    else if(pipeNum == 2)  REG_ADDR = REG_RX_PW_P2;
    else if(pipeNum == 3)  REG_ADDR = REG_RX_PW_P3;
    else if(pipeNum == 4)  REG_ADDR = REG_RX_PW_P4;
    else REG_ADDR = REG_RX_PW_P5;
    return writeRegister(REG_ADDR,&size,1);
}

bool NRF24L01P::setTX_Addr(uint8_t *address) {
    std::copy(address,address+5,TxAddress);
    return writeRegister(REG_TX_ADDR,TxAddress,5);
}

/**
 * @brief set rx address and enable this rx address.
 * @param pipe
 * @param address
 * @return
 */
bool NRF24L01P::openReadingPipe(uint8_t pipe, uint8_t *address) {
    if (!setRX_Addr(pipe, address)) {
        return false;
    }
    uint8_t en_rxaddr{};
    if (!readRegister(REG_EN_RXADDR, &en_rxaddr, 1)) {
        return false;
    }
    en_rxaddr |= (1 << pipe); // Enable data pipe [cite: 774, 775]
    return writeRegister(REG_EN_RXADDR, &en_rxaddr, 1);
}

bool NRF24L01P::PowerDown() {
    uint8_t config{};
    if (!readRegister(REG_CONFIG, &config, 1)) {
        return false;
    }
    config &= ~0x02;
    return writeRegister(REG_CONFIG, &config, 1);
}

bool NRF24L01P::standbyI() {
    write_ce(0);
    uint8_t config{};
    if (!readRegister(REG_CONFIG, &config, 1)) {
        return false;
    }
    config |= 0x02;
    return writeRegister(REG_CONFIG, &config, 1);
}

bool NRF24L01P::start_RxMode() {
    write_ce(0);
    uint8_t config{};
    if (!readRegister(REG_CONFIG, &config, 1)) {
        return false;
    }
    const bool was_powered = (config & 0x02U) != 0U;
    config |= 0x03U;
    const bool isSuccess = writeRegister(REG_CONFIG, &config, 1);
    if (isSuccess && !was_powered) {
        // nRF24L01+ needs up to 1.5 ms from power-down to standby-I.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    write_ce(isSuccess ? 1U : 0U);
    return isSuccess;
}

bool NRF24L01P::start_TxMode() {
    write_ce(0);
    uint8_t config{};
    if (!readRegister(REG_CONFIG, &config, 1)) {
        return false;
    }
    config |= 0x02;
    config &= ~0x01;
    const bool isSuccess = writeRegister(REG_CONFIG, &config, 1);
    write_ce(isSuccess ? 1U : 0U);
    return isSuccess;
}

bool NRF24L01P::send(const uint8_t *data, uint8_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }
    length = std::min(length, PACKET_WIDTH);
    std::array<uint8_t, PACKET_WIDTH + 1> tx{};
    tx[0] = CMD_W_TX_PAYLOAD;
    std::copy_n(data, length, tx.begin() + 1);
    write_ce(0);
    csLow();
    const bool isSuccess = spiSend(tx.data(), length + 1U);
    csHigh();
    return isSuccess && start_TxMode();
}

bool NRF24L01P::getStatus(NRF24L01P::Status_t &status) {
    bool isSuccess;
    uint8_t data{};
    isSuccess = readRegister(REG_STATUS,&data,1);
    if (isSuccess) {
        // Bit 6: Data Ready RX FIFO interrupt
        status.RX_DR   = (data & 0x40U) != 0U;
        // Bit 5: Data Sent TX FIFO interrupt
        status.TX_DS   = (data & 0x20U) != 0U;
        // Bit 4: Maximum number of TX retransmits interrupt
        status.MAX_RT  = (data & 0x10U) != 0U;
        // Bits 3:1: Data pipe number for the payload available for reading
        status.RX_P_NO = static_cast<uint8_t>((data >> 1U) & 0x07U);
        // Bit 0: TX FIFO full flag
        status.TX_FULL = (data & 0x01U) != 0U;
    }
    return isSuccess;
}

bool NRF24L01P::setStatus(uint8_t data) {
    return writeRegister(REG_STATUS,&data,1);
}

/**
 * @brief
 * @param data
 * @param length
 * @return
 */
bool NRF24L01P::tryReceive(uint8_t *data, uint8_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }
    length = std::min(length, PACKET_WIDTH);
    std::array<uint8_t, PACKET_WIDTH + 1> tx{};
    std::array<uint8_t, PACKET_WIDTH + 1> rx{};
    tx[0] = CMD_R_RX_PAYLOAD;
    csLow();
    const bool success = spiTransfer(tx.data(), rx.data(), length + 1U);
    csHigh();
    if (success) {
        std::copy_n(rx.begin() + 1, length, data);
    }
    return success;
}

void NRF24L01P::isrExtiHandler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 向处理 NRF 的任务发送通知
    emitFromISR(IRQEvent_cfg,&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void NRF24L01P::isrSpiDmaCompleteHandler() {
    if (spiDmaSemaphore == nullptr) {
        return;
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(spiDmaSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief
 * @param slot
 */
bool NRF24L01P::signal_IRQEvent(const StatusHandler& slot) {
    if (!slot) {
        return false;
    }

    Status_t status{};
    if (!getStatus(status)) {
        status.IO_ERROR = true;
        static_cast<void>(slot(status));
        return false;
    }

    bool success = true;
    if (status.RX_DR) {
        // Read then clear RX_DR before checking the FIFO. If a packet arrives after
        // the FIFO-empty check it asserts a fresh IRQ instead of being erased by a
        // late W1C, while packets already queued are drained in this service call.
        for (std::uint8_t payload_index = 0;
             payload_index < RX_FIFO_DEPTH;
             ++payload_index) {
            Status_t payload_status = status;
            if (payload_index != 0U) {
                payload_status.TX_DS = false;
                payload_status.MAX_RT = false;
            }
            if (!slot(payload_status)) {
                success = false;
                break;
            }

            success = setStatus(0x40U) && success;
            if (!success) {
                break;
            }

            std::uint8_t fifo_status = 0;
            if (!readRegister(REG_FIFO_STATUS, &fifo_status, 1)) {
                success = false;
                break;
            }
            if ((fifo_status & 0x01U) != 0U) { // RX_EMPTY
                break;
            }
            if (payload_index + 1U == RX_FIFO_DEPTH) {
                // A healthy nRF24L01+ cannot have more than three queued RX
                // payloads. Fail closed if the FIFO never drains so the caller
                // reinitializes and flushes the radio instead of spinning here.
                success = false;
                break;
            }
        }
    }
    else {
        success = slot(status) && success;
    }

    std::uint8_t clear_mask = 0;
    if (status.TX_DS) {
        clear_mask |= 0x20U;
    }
    if (status.MAX_RT) {
        clear_mask |= 0x10U;
        success = flushTx() && success;
    }
    if (clear_mask != 0U) {
        success = setStatus(clear_mask) && success;
    }
    if (status.TX_DS || status.MAX_RT) {
        success = start_RxMode() && success;
    }
    if (!success) {
        Status_t error_status{};
        error_status.IO_ERROR = true;
        static_cast<void>(slot(error_status));
    }
    return success;
}













