/********************************************************************************
  * @file           : HallEncoder.h
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-21
  *******************************************************************************/


#ifndef __HALLENCODER_H
#define __HALLENCODER_H

#include "tim.h"

class HallEncoder {
public:
    ///
    typedef struct{
        uint8_t EncoderLine;        // PPR
        uint8_t ReductionRatio;     //
        uint8_t EdgeCnt;            //
        uint8_t Samplerate;         //
    }InitConfig_t;

private:
    TIM_HandleTypeDef* HTim;
    InitConfig_t Hall_Cfg{};
public:
    ///
    HallEncoder(TIM_HandleTypeDef* _htim, InitConfig_t _cfg);
    ///
    int32_t getCounter();
    double getRPM();
    static int32_t TurnNum_toCnt(const HallEncoder& thisEncoder, float _num);
    static int32_t Rpm_toCnt(const HallEncoder& thisEncoder, float _rpm);
    static float Cnt_toTurnNum(const HallEncoder& thisEncoder, int32_t _num);
};


#endif //__HALLENCODER_H
