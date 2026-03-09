/********************************************************************************
  * @file           : LkUart.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-4
  *******************************************************************************/


#ifndef __LKUART_HPP
#define __LKUART_HPP

//c++ standard library include
#include <format>
// stm-Hal library include
#include "main.h"

class LkUart {
    static constexpr uint16_t SEND_BUF_SIZE = 128;
private:
    UART_HandleTypeDef* HUart;
public:
    explicit LkUart(UART_HandleTypeDef* _huart);
    inline void UART_SendString(const char* str, size_t len);
    //
//    template <typename... Args>
//    void print(const std::format_string<Args...> fmt, Args&&... args){
//        char buf[SEND_BUF_SIZE]; // 根据需求调整大小
//        auto res = std::format_to_n(buf, sizeof(buf) - 1, fmt, std::forward<Args>(args)...);
//        *res.out = '\0';
//        UART_SendString(buf, res.size);
//    }
    //

};


#endif //__LKUART_HPP
