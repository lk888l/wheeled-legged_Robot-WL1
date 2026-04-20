/********************************************************************************
  * @file           : TB6612.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @version        : v1.0
  * @date           : 2/17/2026
  *******************************************************************************/

/// cpp standard library include
#include <cstdlib>
/// user library include
#include "TB6612.h"


TB6612::TB6612(InitConfig_t _tbconfig)
    : TB6_Cfg(_tbconfig)
{

}

/**
 * @brief start PWM
 * @return true: successful  or   false: fail
 */
bool TB6612::Init() const
{
    uint8_t HalState;
    HalState = HAL_TIM_PWM_Start(TB6_Cfg.Htim,TB6_Cfg.AChannel);
    HalState |= HAL_TIM_PWM_Start(TB6_Cfg.Htim,TB6_Cfg.BChannel);
    if(HalState == HAL_OK){
        return true;
    }
    return false;
}

/**
 * @brief set the positive and negative directions of motion
 * @param target A or B or A|B
 * @param dirParam Negative or positive
 */
void TB6612::setDirection_Cfg(uint8_t target, TB6612::Direction dirParam)
{
    if(target & static_cast<uint8_t>(OutPort::A)){
        ADIR_Mask = static_cast<uint8_t>(dirParam);
    }
    if(target & static_cast<uint8_t>(OutPort::B)){
        BDIR_Mask = static_cast<uint8_t>(dirParam);
    }
}

/**
 * @brief
 * @param _value
 */
void TB6612::setA_DeadZone(uint16_t _value) {
    ADeadZone = _value;
}

void TB6612::setB_DeadZone(uint16_t _value) {
    BDeadZone = _value;
}

/**
 * @brief set
 * @param _dir
 */
void TB6612::setADirection(uint8_t _dir) {
    _dir = !!_dir;
    if(_dir){
        HAL_GPIO_WritePin(TB6_Cfg.A1GPIO_Port, TB6_Cfg.A1GPIO_Pin, static_cast<GPIO_PinState>(ADIR_Mask));
        HAL_GPIO_WritePin(TB6_Cfg.A2GPIO_Port, TB6_Cfg.A2GPIO_Pin, static_cast<GPIO_PinState>(ADIR_Mask^1));
    }
    else{
        HAL_GPIO_WritePin(TB6_Cfg.A1GPIO_Port, TB6_Cfg.A1GPIO_Pin, static_cast<GPIO_PinState>(ADIR_Mask^1));
        HAL_GPIO_WritePin(TB6_Cfg.A2GPIO_Port, TB6_Cfg.A2GPIO_Pin, static_cast<GPIO_PinState>(ADIR_Mask));
    }
}

void TB6612::setBDirection(uint8_t _dir) {
    _dir = !!_dir;
    if(_dir){
        HAL_GPIO_WritePin(TB6_Cfg.B1GPIO_Port, TB6_Cfg.B1GPIO_Pin, static_cast<GPIO_PinState>(BDIR_Mask));
        HAL_GPIO_WritePin(TB6_Cfg.B2GPIO_Port, TB6_Cfg.B2GPIO_Pin, static_cast<GPIO_PinState>(BDIR_Mask^1));
    }
    else{
        HAL_GPIO_WritePin(TB6_Cfg.B1GPIO_Port, TB6_Cfg.B1GPIO_Pin, static_cast<GPIO_PinState>(BDIR_Mask^1));
        HAL_GPIO_WritePin(TB6_Cfg.B2GPIO_Port, TB6_Cfg.B2GPIO_Pin, static_cast<GPIO_PinState>(BDIR_Mask));
    }
}

/**
 * @brief
 * @param _value
 * @return true: successful  or   false: fail
 */
void TB6612::setAPWM(uint16_t _value) {
    __HAL_TIM_SET_COMPARE(TB6_Cfg.Htim, TB6_Cfg.AChannel, _value);
}

/**
 * @brief
 * @param _value
 * @return true: successful  or   false: fail
 */
void TB6612::setBPWM(uint16_t _value) {
    __HAL_TIM_SET_COMPARE(TB6_Cfg.Htim, TB6_Cfg.BChannel, _value);
}

void TB6612::setAVel_raw(int16_t _value) {
    if(_value>0) {
//        _value += ADeadZone;
        setADirection(1);
    }
    else if(_value<0){
//        _value -= ADeadZone;
        setADirection(0);
    }
    else{
        //stop
        setAPWM(0);
    }
    _value = std::abs(_value);
    if(_value < ADeadZone)
    {_value = ADeadZone;}
    setAPWM(_value);
}

void TB6612::setBVel_raw(int16_t _value) {
    if(_value>0) {
//        _value += BDeadZone;
        setBDirection(1);
    }
    else if(_value<0){
//        _value -= BDeadZone;
        setBDirection(0);
    }
    else{
        //stop
        setBPWM(0);
    }
    _value = std::abs(_value);
    if(_value < BDeadZone)
    {_value = BDeadZone;}
    setBPWM(_value);
}

TB6612::~TB6612()
{
    HAL_TIM_PWM_Stop(TB6_Cfg.Htim,TB6_Cfg.AChannel);
    HAL_TIM_PWM_Stop(TB6_Cfg.Htim,TB6_Cfg.BChannel);
}




