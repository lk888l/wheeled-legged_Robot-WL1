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

// 宏函数：频率(Hz)转周期(ms)
// 注意：宏是文本替换，无类型检查，需确保输入f>0
#define HZ_TO_MS_MACRO(f) (1000.0 / (f))
#define MS_TO_HZ_MACRO(ms) (1000.0 / (ms))

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
    uint32_t lastRawTick{};         //
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

    /**
     * @brief
     * @param frequency_hz
     * @return
     */
    static inline double HZ_toms(double frequency_hz) {
        // 防除零错误：频率不能为0或负数
//        if (frequency_hz <= 0) {
//            return 0;
//        }
        return 1000.0 / frequency_hz;
    }

    /**
     * @brief
     * @param period_ms
     * @return
     */
    static inline double ms_toHZ(double period_ms) {
        // 防除零错误：周期不能为0或负数（无意义）
//        if (period_ms <= 0) {
//            return 0;
//        }
        return 1000.0 / period_ms;
    }
};


#endif //__HALLENCODER_H
