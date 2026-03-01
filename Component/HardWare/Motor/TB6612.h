/********************************************************************************
  * @file           : TB6612.h
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @version        : v1.0
  * @date           : 2/17/2026
  *******************************************************************************/


#ifndef __TB6612MOTOR_H
#define __TB6612MOTOR_H

//#define EN_SETVEL_PERCENT

#include <memory>
#include "tim.h"



class TB6612 {
public:
    /**
     * @brief
     */
    typedef struct{
        TIM_HandleTypeDef*  Htim;          /*  PWM timer handle such as: &htim2*/
        uint32_t            AChannel;      /*  */
        uint32_t            BChannel;      /*  */
        GPIO_TypeDef*       A1GPIO_Port;  /*  */
        uint16_t            A1GPIO_Pin;   /*  */
        GPIO_TypeDef*       A2GPIO_Port;  /*  */
        uint16_t            A2GPIO_Pin;   /*  */
        GPIO_TypeDef*       B1GPIO_Port;  /*  */
        uint16_t            B1GPIO_Pin;   /*  */
        GPIO_TypeDef*       B2GPIO_Port;  /*  */
        uint16_t            B2GPIO_Pin;   /*  */
    }InitConfig_t;

    enum class OutPort : uint8_t {
        A = 0x01,
        B = 0x02,
    };

    enum class Direction : uint8_t {
        Negative = 0,
        Positive = 1
    };

private:
    ///
    InitConfig_t TB6_Cfg{};
    char ADIR_Mask{1};
    char BDIR_Mask{1};
    /// private member function
    void setADirection(uint8_t _dir);
    void setBDirection(uint8_t _dir);

public:
    /// constructor
    explicit TB6612(InitConfig_t _tbconfig);
    TB6612(const TB6612&) = delete;
    TB6612& operator=(const TB6612&) = delete;
    /// member function
    bool Init() const;
    void setAPWM(uint16_t _value);
    void setBPWM(uint16_t _value);
    void setDirection_Cfg(uint8_t target, Direction dirParam);
    void setAVel_raw(int16_t _value);
    void setBVel_raw(int16_t _value);

#ifdef EN_SETVEL_PERCENT
    bool setAVel_percent(float _percent);
    bool setBVel_percent(float _percent);
#endif //EN_SETVEL_PERCENT

    ~TB6612();
};


#endif //__TB6612MOTOR_H
