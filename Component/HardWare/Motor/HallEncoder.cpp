/********************************************************************************
  * @file           : HallEncoder.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-21
  *******************************************************************************/


#include "HallEncoder.h"



HallEncoder::HallEncoder(TIM_HandleTypeDef *_htim, HallEncoder::InitConfig_t _cfg)
    :HTim(_htim)
    , Hall_Cfg(_cfg)
{
    HAL_TIM_Encoder_Start(_htim,TIM_CHANNEL_ALL);
}

/**
 * @brief
 * @return
 */
int32_t HallEncoder::getCounter() {
    int32_t val = __HAL_TIM_GET_COUNTER(HTim);
    AccumulateNum += val;
    __HAL_TIM_SET_COUNTER(HTim,0);
    return val;
}

/**
 * @brief rad/min
 * @return
 */
double HallEncoder::getRPM() {
    return ((getCounter() / 1.0 / Hall_Cfg.EdgeCnt / Hall_Cfg.EncoderLine / Hall_Cfg.ReductionRatio) * (1000.0/Hall_Cfg.Samplerate)*60);
    // 分步计算，提高可读性，保留精度
//    double perTurnCnt = Hall_Cfg.EdgeCnt * Hall_Cfg.EncoderLine * Hall_Cfg.ReductionRatio;
//    double turnsPerMs = getCounter() / 1.0 / perTurnCnt;
//    double turnsPerSecond = turnsPerMs * 1000.0 / Hall_Cfg.Samplerate;
//    return turnsPerSecond * 60; // 转换为RPM
}

void HallEncoder::clearCounter() {
    AccumulateNum = 0;
    __HAL_TIM_SET_COUNTER(HTim,0);
}

int64_t HallEncoder::getAccumCnt() {
    return AccumulateNum;
}

int32_t HallEncoder::TurnNum_toCnt(const HallEncoder &thisEncoder, float _num) {
    return (int32_t)(_num * thisEncoder.Hall_Cfg.EdgeCnt * thisEncoder.Hall_Cfg.EncoderLine * thisEncoder.Hall_Cfg.ReductionRatio);
}

int32_t HallEncoder::Rpm_toCnt(const HallEncoder &thisEncoder, float _rpm) {
    return (int32_t)((_rpm / 60.0) * thisEncoder.Hall_Cfg.Samplerate / 1000.0) * thisEncoder.Hall_Cfg.EdgeCnt * thisEncoder.Hall_Cfg.EncoderLine * thisEncoder.Hall_Cfg.ReductionRatio;
}

float HallEncoder::Cnt_toTurnNum(const HallEncoder &thisEncoder, int32_t _num) {
    return (static_cast<float>(_num) / thisEncoder.Hall_Cfg.EdgeCnt / thisEncoder.Hall_Cfg.EncoderLine / thisEncoder.Hall_Cfg.ReductionRatio);
}




