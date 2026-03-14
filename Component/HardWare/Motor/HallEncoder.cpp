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
    , lastRawTick(0)
{
    HAL_TIM_Encoder_Start(_htim,TIM_CHANNEL_ALL);
}

/**
 * @brief 获取增量计数（解决16位/32位溢出问题）
 * @return 距离上次调用后的脉冲增量（带正负号）
 */
int32_t HallEncoder::getCounter() {
    uint32_t currentRawTick = __HAL_TIM_GET_COUNTER(HTim);
    int32_t delta{};
    if (HTim->Instance == TIM2) {
        // TIM2 是 32位
        // 直接做差，利用 uint32_t 的溢出特性自动处理回环
        delta = static_cast<int32_t>(currentRawTick - lastRawTick);
    } else {
        // TIM3 是 16位
        // 强制转换为 uint16_t 做差后再转回 int16_t 即可处理 65535->0 的跳转
        delta = static_cast<int16_t>(static_cast<uint16_t>(currentRawTick) -
                                     static_cast<uint16_t>(lastRawTick));
    }

    lastRawTick = currentRawTick; // 更新记录
    AccumulateNum += delta;      // 更新总计数值
    return delta;
}

/**
 * @brief rad/min
 * @attention : use getCounter Counter will set zero
 * @return
 */
double HallEncoder::getRPM() {
    int32_t delta = getCounter();
    // 增加分母检查，防止除零
    double perTurnCnt = static_cast<double>(Hall_Cfg.EdgeCnt) * Hall_Cfg.EncoderLine * Hall_Cfg.ReductionRatio;
//    if (perTurnCnt == 0) return 0;
    // 计算公式：(增量 / 每圈脉冲数) * (60s / 采样周期s)
    return (delta / perTurnCnt) * (60000.0 / Hall_Cfg.Samplerate);
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




