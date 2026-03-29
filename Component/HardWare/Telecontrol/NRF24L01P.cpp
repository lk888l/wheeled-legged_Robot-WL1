/********************************************************************************
  * @file           : NRF24L01P.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-27
  *******************************************************************************/


#include "NRF24L01P.hpp"
#include <algorithm>

NRF24L01P::NRF24L01P(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *cePort,
                     uint16_t cePin) {

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

bool NRF24L01P::spiSend(const uint8_t *pData, uint16_t size) {
    return static_cast<bool>(HAL_SPI_Transmit(HSpi, pData, size, HAL_MAX_DELAY));
}

bool NRF24L01P::spiReceive(uint8_t *pData, uint16_t size) {
    return static_cast<bool>(HAL_SPI_Receive(HSpi, pData, size, HAL_MAX_DELAY));
}

bool NRF24L01P::spiTransfer(const uint8_t *pTxData, uint8_t *pRxData, uint16_t size) {
    return static_cast<bool>(HAL_SPI_TransmitReceive(HSpi, pTxData, pRxData, size, HAL_MAX_DELAY));
}

bool NRF24L01P::readRegister(uint8_t reg, uint8_t *buf, uint8_t len) {
    csLow();
    bool isSuccess= true;
    uint8_t regAddress = CMD_R_REGISTER | (reg & 0x1F);
    isSuccess &= spiSend(&regAddress,1);
    isSuccess &= spiReceive(buf,len);
    csHigh();
    return isSuccess;
}

bool NRF24L01P::writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len){
    csLow();
    bool isSuccess= true;
    uint8_t regAddress = CMD_R_REGISTER | (reg & 0x1F);
    isSuccess &= spiSend(&regAddress,1);
    isSuccess &= spiSend(buf,len);
    csHigh();
    return isSuccess;
}

bool NRF24L01P::flushTx() {
    csLow();
    bool isSuccess;
    uint8_t command = CMD_FLUSH_TX;
    isSuccess = spiSend(&command,1);  // Flush RX FIFO, used in RX mode [cite: 711]
    csHigh();
    return isSuccess;
}

bool NRF24L01P::flushRx() {
    csLow();
    bool isSuccess;
    uint8_t command = CMD_FLUSH_RX;
    isSuccess = spiSend(&command,1);  // Flush RX FIFO, used in RX mode [cite: 711]
    csHigh();
    return isSuccess;
}

/**
 * @brief
 */
bool NRF24L01P::Init() {
//    ceLow();
//    csHigh();
//    HAL_Delay(100); // Wait for Power on reset

    bool isSuccess = true;
    uint8_t command;
    // Set configuration: Enable 2 byte CRC, Power Up, PTX mode [cite: 770, 772]
    command = 0x0A;
    isSuccess &= writeRegister(REG_CONFIG, &command,1); // EN_CRC=1, CRCO=0 (1 byte), PWR_UP=1, PRIM_RX=0
    //Set EN_AA auto-answer number of channels (open all)
    command = 0x3F;
    isSuccess &= writeRegister(REG_EN_AA,&command,1);
    //Set Receive channel(open all)
    command = 0x03;
    isSuccess &= writeRegister(REG_EN_RXADDR,&command,1);
    //Set Address Widths
    command = 0x03;
    isSuccess &= writeRegister(REG_SETUP_AW,&command,1);
    // Set Auto Retransmit Delay to 250us and Up to 3 Re-Transmits [cite: 775]
    command = (0x00 << 4) | 0x03;
    isSuccess &= writeRegister(REG_SETUP_RETR, &command, 1);
    isSuccess &= setChannel(66);
    isSuccess &= setDataRate(2); // Default to 2Mbps
    isSuccess &= setPALevel(3);  // Default to 0dBm

    isSuccess &= setRX_Addr(0,RxAddress_P0);
    isSuccess &= setRX_PW(0,4);
    isSuccess &= setTX_Addr(TxAddress);
    flushRx();
    flushTx();
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
    uint8_t setup;
    bool isSuccess;
    isSuccess = readRegister(REG_RF_SETUP,&setup,1);
    setup &= (~0x06);
    if (level > 3) level = 3;
    setup |= (level << 1);
    isSuccess &= writeRegister(REG_RF_SETUP, &setup,1);
    return isSuccess;
}

/**
 * @brief Encoding: '00' 1Mbps, '01' 2Mbps, '10' 250kbps [cite: 781]
 * @param rate
 * @return
 */
bool NRF24L01P::setDataRate(uint8_t rate) {
    uint8_t setup;
    bool isSuccess;
    isSuccess = readRegister(REG_RF_SETUP,&setup,1); // Clear RF_DR_LOW and RF_DR_HIGH
    setup  &= (~0x28);
    if (rate == 0) setup |= (1 << 5);       // 250kbps (RF_DR_LOW = 1)
    else if (rate == 1) setup &= ~(1 << 3); // 1Mbps (RF_DR_HIGH = 0)
    else if (rate == 2) setup |= (1 << 3);  // 2Mbps (RF_DR_HIGH = 1)
    isSuccess&=writeRegister(REG_RF_SETUP, &setup,1);
    return isSuccess;
}


bool NRF24L01P::setRX_Addr(uint8_t pipeNum,uint8_t *address) {
    uint8_t REG_ADDR;
    if(pipeNum>5) pipeNum=5;
    if(pipeNum == 0)  {
        REG_ADDR=  REG_RX_ADDR_P0;
        std::copy(RxAddress_P0,RxAddress_P0+5,address);
    }
    else if(pipeNum == 1)  REG_ADDR = REG_RX_ADDR_P1;
    else if(pipeNum == 2)  REG_ADDR = REG_RX_ADDR_P2;
    else if(pipeNum == 3)  REG_ADDR = REG_RX_ADDR_P3;
    else if(pipeNum == 4)  REG_ADDR = REG_RX_ADDR_P4;
    else REG_ADDR = REG_RX_ADDR_P5;
    return writeRegister(REG_ADDR,address,5);
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
    return writeRegister(REG_ADDR,&size,5);
}

bool NRF24L01P::setTX_Addr(uint8_t *address) {
    std::copy(TxAddress,TxAddress+5,address);
    return writeRegister(REG_TX_ADDR,TxAddress,5);
}

/**
 * @brief set rx address and enable this rx address.
 * @param pipe
 * @param address
 * @return
 */
bool NRF24L01P::openReadingPipe(uint8_t pipe, uint8_t *address) {
    bool isSuccess = true;
    setRX_Addr(pipe,address);
    uint8_t en_rxaddr;
    isSuccess = readRegister(REG_EN_RXADDR,&en_rxaddr,1);
    en_rxaddr |= (1 << pipe); // Enable data pipe [cite: 774, 775]
    writeRegister(REG_EN_RXADDR, &en_rxaddr, 1);
    return isSuccess;
}

bool NRF24L01P::PowerDown() {
    bool isSuccess;
    uint8_t config;
    isSuccess = readRegister(REG_CONFIG,&config,1);
    config &= ~0x02;
    isSuccess &= writeRegister(REG_CONFIG,&config,1);
    return isSuccess;
}

bool NRF24L01P::standbyI() {
    write_ce(0);
    bool isSuccess;
    uint8_t config;
    isSuccess = readRegister(REG_CONFIG,&config,1);
    config |= 0x02;
    isSuccess &= writeRegister(REG_CONFIG,&config,1);
    return isSuccess;
}

bool NRF24L01P::start_RxMode() {
    write_ce(0);
    bool isSuccess;
    uint8_t config{};
    isSuccess = readRegister(REG_CONFIG,&config,1);
    config |= 0x03;
    isSuccess &= writeRegister(REG_CONFIG,&config,1);
    write_ce(1);
    return isSuccess;
}

bool NRF24L01P::start_TxMode() {
    write_ce(0);
    bool isSuccess;
    uint8_t config{};
    isSuccess = readRegister(REG_CONFIG,&config,1);
    config |= 0x02;
    config &= ~0x01;
    isSuccess &= writeRegister(REG_CONFIG,&config,1);
    write_ce(1);
    return isSuccess;
}

bool NRF24L01P::send(const uint8_t *data, uint8_t length) {
    bool isSuccess;
    uint8_t command = CMD_W_TX_PAYLOAD;
    csLow();
    isSuccess = spiSend(&command,1);
    isSuccess &= spiSend(data,length);
    csHigh();
    isSuccess &= start_TxMode();
    return isSuccess;
}

bool NRF24L01P::getStatus(NRF24L01P::Status_t &status) {
    bool isSuccess;
    uint8_t data{};
    isSuccess = readRegister(REG_STATUS,&data,1);
    if (isSuccess) {
        // Bit 6: Data Ready RX FIFO interrupt
        status.RX_DR   = (data & 0x40) >> 6;
        // Bit 5: Data Sent TX FIFO interrupt
        status.TX_DS   = (data & 0x20) >> 5;
        // Bit 4: Maximum number of TX retransmits interrupt
        status.MAX_RT  = (data & 0x10) >> 4;
        // Bits 3:1: Data pipe number for the payload available for reading
        status.RX_P_NO = (data & 0x0E) >> 1;
        // Bit 0: TX FIFO full flag
        status.TX_FULL = (data & 0x01);
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
bool NRF24L01P::tryRecive(uint8_t *data, uint8_t length) {
    Status_t status;
    uint8_t command = CMD_R_RX_PAYLOAD;
    getStatus(status);
    if(status.RX_DR){
        csLow();
        spiSend(&command,1);
        spiSend(data,length);
        csHigh();
        setStatus(0x40);
        flushRx();
        return true;
    } else{
        return false;
    }
}










