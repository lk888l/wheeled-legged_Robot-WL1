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
    static constexpr double PI = 3.14159265358979323846;

    ///
    struct InitConfig_t{
        uint8_t EncoderLine;        // PPR
        uint8_t ReductionRatio;     //
        uint8_t EdgeCnt;            //
        uint8_t Samplerate;         //ms
    };


private:
    TIM_HandleTypeDef* HTim = nullptr;
    InitConfig_t Hall_Cfg{};
    int64_t AccumulateNum{};
public:
    ///
    HallEncoder(TIM_HandleTypeDef* _htim, InitConfig_t _cfg);
    ///
    int32_t getCounter();
    double getRPM();
    void clearCounter();
    int64_t getAccumCnt();
    static int32_t TurnNum_toCnt(const HallEncoder& thisEncoder, float _num);
    static int32_t Rpm_toCnt(const HallEncoder& thisEncoder, float _rpm);
    static float Cnt_toTurnNum(const HallEncoder& thisEncoder, int32_t _num);
    static inline double Rpm_ToMS(double WheelRadius, double _n)
    {
        return _n * PI * WheelRadius * 2;
    }

};


#endif //__HALLENCODER_H
