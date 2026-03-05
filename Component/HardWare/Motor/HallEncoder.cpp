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

}


int32_t HallEncoder::getCounter() {
    int32_t val = __HAL_TIM_GET_COUNTER(HTim);
    __HAL_TIM_SET_COUNTER(HTim,0);
    return val;
}

double HallEncoder::getRPM() {
    return ((getCounter() / 1.0 / Hall_Cfg.EdgeCnt / Hall_Cfg.EncoderLine / Hall_Cfg.ReductionRatio) * (1000/Hall_Cfg.Samplerate)*60);
}

int32_t HallEncoder::TurnNum_toCnt(const HallEncoder &thisEncoder, float _num) {
    return (int32_t)(_num * thisEncoder.Hall_Cfg.EdgeCnt * thisEncoder.Hall_Cfg.EncoderLine * thisEncoder.Hall_Cfg.ReductionRatio);
}

int32_t HallEncoder::Rpm_toCnt(const HallEncoder &thisEncoder, float _rpm) {
    return (int32_t)(_rpm/60/1000 *thisEncoder.Hall_Cfg.Samplerate * thisEncoder.Hall_Cfg.EdgeCnt * thisEncoder.Hall_Cfg.EncoderLine * thisEncoder.Hall_Cfg.ReductionRatio);
}

float HallEncoder::Cnt_toTurnNum(const HallEncoder &thisEncoder, int32_t _num) {
    return (static_cast<float>(_num) / thisEncoder.Hall_Cfg.EdgeCnt / thisEncoder.Hall_Cfg.EncoderLine / thisEncoder.Hall_Cfg.ReductionRatio);
}

