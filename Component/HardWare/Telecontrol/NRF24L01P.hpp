/********************************************************************************
  * @file           : NRF24L01P.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-27
  *******************************************************************************/

#ifdef __GNUC__
#pragma once
#endif //__GNUC__
#ifndef __NRF24L01P_HPP
#define __NRF24L01P_HPP

//stm-hal library include
#include <string_view>
#include "main.h"
//User-lk library include
#include "BasicObject.hpp"
#include "etl/string_view.h"
//freeRTOS variable define
#include "semphr.h"


class NRF24L01P : BasicObject{
public:
    // SPI Commands [cite: 704, 705, 707, 709, 711]
    static constexpr int CMD_R_REGISTER   =       0x00;
    static constexpr int CMD_W_REGISTER   =       0x20;
    static constexpr int CMD_R_RX_PAYLOAD =       0x61;
    static constexpr int CMD_W_TX_PAYLOAD =       0xA0;
    static constexpr int CMD_FLUSH_TX     =       0xE1;
    static constexpr int CMD_FLUSH_RX     =       0xE2;
    static constexpr int CMD_REUSE_TX_PL  =       0xE3;
    static constexpr int CMD_R_RX_PL_WID  =       0x60;
    static constexpr int CMD_NOP          =       0xFF;
    // Register Map [cite: 770, 772, 774, 775, 781, 783, 799, 807, 809, 817]
    static constexpr int REG_CONFIG      =        0x00;
    static constexpr int REG_EN_AA       =        0x01;
    static constexpr int REG_EN_RXADDR   =        0x02;
    static constexpr int REG_SETUP_AW    =        0x03;
    static constexpr int REG_SETUP_RETR  =        0x04;
    static constexpr int REG_RF_CH       =        0x05;
    static constexpr int REG_RF_SETUP    =        0x06;
    static constexpr int REG_STATUS      =        0x07;
    static constexpr int REG_OBSERVE_TX  =        0x08;
    static constexpr int REG_RPD         =        0x09;
    static constexpr int REG_RX_ADDR_P0  =        0x0A;
    static constexpr int REG_RX_ADDR_P1  =        0x0B;
    static constexpr int REG_RX_ADDR_P2  =        0x0C;
    static constexpr int REG_RX_ADDR_P3  =        0x0D;
    static constexpr int REG_RX_ADDR_P4  =        0x0E;
    static constexpr int REG_RX_ADDR_P5  =        0x0F;
    static constexpr int REG_TX_ADDR     =        0x10;
    static constexpr int REG_RX_PW_P0    =        0x11;
    static constexpr int REG_RX_PW_P1    =        0x12;
    static constexpr int REG_RX_PW_P2    =        0x13;
    static constexpr int REG_RX_PW_P3    =        0x14;
    static constexpr int REG_RX_PW_P4    =        0x15;
    static constexpr int REG_RX_PW_P5    =        0x16;
    static constexpr int REG_FIFO_STATUS =        0x17;
    static constexpr int REG_DYNPD       =        0x1C;
    static constexpr int REG_FEATURE     =        0x1D;
    /// User define
    static constexpr uint8_t PACKET_WIDTH =       32;
public:
    /**
     * @brief nRF24L01+ STATUS (0x07) 寄存器结构体
     * 使用位域直接映射 8-bit 寄存器内容
     */
    struct Status_t{
        uint8_t TX_FULL {};  // [位0] TX FIFO 满标志。1: 已满；0: 尚有空间。
        uint8_t RX_P_NO {};  // [位1-3] 接收数据通道号。000-101 表示通道0-5；110 未使用；111 表示 RX FIFO 为空。
        uint8_t MAX_RT  {};  // [位4] 达到最大重发次数中断。若此位为1，必须手动清除才能继续通信。
        uint8_t TX_DS   {};  // [位5] 数据发送完成中断（收到 ACK 后置位）。
        uint8_t RX_DR   {};  // [位6] 收到有效数据中断。
    };

    NRF24L01P(SPI_HandleTypeDef* hspi,
              GPIO_TypeDef* csPort, uint16_t csPin,
              GPIO_TypeDef* cePort, uint16_t cePin,
              GPIO_TypeDef* irqPort, uint16_t irqPin);
    // Initialization and Configuration
    bool Init();
    SPI_HandleTypeDef* getSPIHandle();
    uint16_t getIRQGPIOPort();
    bool setChannel(uint8_t channel);
    bool setPALevel(uint8_t level);
    bool setDataRate(uint8_t rate);
    bool setRX_Addr(uint8_t pipeNum,uint8_t address[5]);
    bool setRX_PW(uint8_t pipeNum, uint8_t size);
    bool setTX_Addr(uint8_t address[5]);

    // Operating Modes [cite: 320, 354, 342]
//    void startListening();
//    void stopListening();
    // Communication
    bool openReadingPipe(uint8_t pipe, uint8_t* address);
    bool PowerDown();
    bool standbyI();
    bool start_RxMode();
    bool start_TxMode();
    bool send(const uint8_t* data, uint8_t length);
    bool getStatus(Status_t &status);
    bool setStatus(uint8_t data);
    bool tryReceive(uint8_t* data, uint8_t length = PACKET_WIDTH);
    bool flushRx();
    bool flushTx();

    //signal bindReactor
    template<typename SignalPtr>
    bool bindReactor(SignalPtr signalFunc, TaskHandle_t task, uint32_t bitMask) {
        if constexpr (std::is_same_v<SignalPtr, decltype(&NRF24L01P::signal_IRQEvent)>) {
            if (signalFunc == &NRF24L01P::signal_IRQEvent) {
                IRQEvent_cfg.task_h = task;
                IRQEvent_cfg.bitMask = bitMask;
                return true;
            }
        }
        return false;
    }
    /**
     * @brief 供外部 EXTI 中断回调调用的函数
     */
    void isrExtiHandler();
    /**
     * @brief 供外部 SPI DMA 完成中断调用的函数
     */
    void isrSpiDmaCompleteHandler();
    /* signal function define */
    void signal_IRQEvent(std::function<void(Status_t &curStatus)> slot);

    /**
     * @brief 将字符串数据填充到uint8_t数组
     * @param sv
     * @param NRF_Tx_Text
     */
    static inline void str_touint8(const etl::string_view &sv, uint8_t *NRF_Text) {
        if (NRF_Text == nullptr) return;
        size_t copy_len = std::min(sv.size(), size_t(PACKET_WIDTH-1));
        if(copy_len != 31)  {memset(&NRF_Text[copy_len], 0, PACKET_WIDTH-copy_len);}
        else {NRF_Text[PACKET_WIDTH] = 0;}
        std::copy(sv.begin(), sv.begin() + copy_len, NRF_Text);
    }
    /**
     * @brief 将uint8_t数组填充到字符串
     * @param sv
     * @param NRF_Text
     */
    static inline void uint8_tostr(etl::string_view &sv, uint8_t *NRF_Text){
        const char* data_ptr = reinterpret_cast<const char*>(NRF_Text);
        size_t actual_len = 0;
        while (actual_len < 32 && NRF_Text[actual_len] != '\0') {
            actual_len++;
        }
        sv = etl::string_view(data_ptr, actual_len);
    }

    /**
     * @brief
     * @param buffer
     * @param args
     */
    static void args_touint8s(uint8_t *buffer, const volatile float* args[4]){
        char* ptr = reinterpret_cast<char*>(buffer);
        char* const last = reinterpret_cast<char*>(buffer) + 32; // 缓冲区绝对终点

        for (uint8_t i=0; i<4; i++){
            if (ptr + 8 >= last) break;
            *ptr++ = ' ';
            float val{};
            if(args[i] != nullptr) {val = *args[i];}
            if (val < 0) {
                *ptr++ = '-';
                val = -val;
            }
            uint32_t total_dec = static_cast<uint32_t>(val * 10.0f + 0.5f);
            uint32_t integer_part = total_dec / 10;
            uint32_t fractional_part = total_dec % 10;
            auto res = std::to_chars(ptr, reinterpret_cast<char *>(buffer - 4), integer_part);
            if (res.ec == std::errc()) {
                ptr = res.ptr;
                *ptr++ = '.';
                *ptr++ = static_cast<char>('0' + fractional_part);
            }else {
                // 空间不足，在此可做异常处理
                return;
            }
        }
        *++ptr = '\0';
    }

private:
    SPI_HandleTypeDef*  HSpi;
    GPIO_TypeDef*       csPort;
    uint16_t            csPin;
    GPIO_TypeDef*       cePort;
    uint16_t            cePin;
    GPIO_TypeDef*       irqPort;
    uint16_t            irqPin;
    uint8_t             RxAddress_P0[5] = {0x11,0x52,0x01,0x31,0x41};
    uint8_t             TxAddress[5] = {0x11,0x52,0x01,0x31,0x41};
    //freeRTOS variable define
    SemaphoreHandle_t   spiDmaSemaphore = nullptr;
    /* signal config define */
    SignalContext IRQEvent_cfg{};
    // Low-level SPI and Pin operations
    void csLow();
    void csHigh();
    void write_ce(uint8_t gpio_status);
    inline bool spiSend(const uint8_t *pData, uint16_t size);
    inline bool spiReceive(uint8_t *pData, uint16_t size);
    inline bool spiTransfer(const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
    // Register operations [cite: 704, 705]
    bool readRegister(uint8_t reg, uint8_t* buf, uint8_t len);
    bool writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len);

};


#endif //WL1_F411CEU6_NRF24L01P_HPP
