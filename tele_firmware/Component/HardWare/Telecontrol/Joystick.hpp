/********************************************************************************
  * @file           : Joystick.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-4-20
  *******************************************************************************/


#ifndef __TELE_JOYSTICK_HPP
#define __TELE_JOYSTICK_HPP

#include <algorithm>
#include "BasicObject.hpp"

class Joystick : BasicObject{
private:
    uint16_t adc_min;       // ADC 最小值 (通常是 0)
    uint16_t adc_max;       // ADC 最大值 (12位ADC通常是 4095)
    uint16_t adc_center;    // 物理中位时的 ADC 值 (通常在 2048 左右)
    uint16_t deadzone;      // 死区范围 (+/- 这个值以内的偏差视为 0)

    float out_min;          // 输出转换后的最小值 (例如 -100.0)
    float out_max;          // 输出转换后的最大值 (例如 100.0)
    /* signal config define */
    SignalContext Complete_cfg{};
public:
    // 构造函数：初始化摇杆参数
    Joystick(uint16_t min_v, uint16_t max_v, uint16_t center_v, uint16_t dz, float o_min, float o_max)
            : adc_min(min_v), adc_max(max_v), adc_center(center_v), deadzone(dz), out_min(o_min), out_max(o_max) {}
    // 核心转换函数：输入原始ADC，输出指定范围的浮点数值
    // 核心转换函数：输入原始ADC，输出指定范围的浮点数值
    float get_converted_value(uint16_t raw_adc) {
        // 1. 边界限幅，防止 ADC 抖动超出预期范围
        raw_adc = std::max(adc_min, std::min(raw_adc, adc_max));

        // 2. 计算相对中心的偏移量
        int32_t offset = raw_adc - adc_center;

        // 【新增】计算目标输出区间的理论中间值
        float out_center = (out_min + out_max) / 2.0f;

        // 3. 死区判断
        if (std::abs(offset) <= deadzone) {
            return out_center; // 在死区内，直接输出该区间的中间值
        }

        // 4. 线性映射到目标范围
        if (offset > 0) {
            // 摇杆往正方向推 (跳过死区部分)
            float range_in = adc_max - (adc_center + deadzone);
            float range_out = out_max - out_center; // 映射跨度变为：中心值到最大值

            // 防止除以零引发硬件 HardFault
            if (range_in <= 0) return out_max;

            // 以 out_center 为起点进行累加
            return out_center + ((raw_adc - (adc_center + deadzone)) / range_in) * range_out;
        } else {
            // 摇杆往负方向推 (跳过死区部分)
            float range_in = (adc_center - deadzone) - adc_min;
            float range_out = out_center - out_min; // 映射跨度变为：最小值到中心值

            if (range_in <= 0) return out_min;

            // 以 out_min 为起点进行累加
            return out_min + ((raw_adc - adc_min) / range_in) * range_out;
        }
    }


    //signal bindReactor
    template<typename SignalPtr>
    bool bindReactor(SignalPtr signalFunc, TaskHandle_t task, uint32_t bitMask) {
        if constexpr (std::is_same_v<SignalPtr, decltype(&Joystick::signal_complete)>) {
            if (signalFunc == &Joystick::signal_complete) {
                Complete_cfg.task_h = task;
                Complete_cfg.bitMask = bitMask;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief
     * @param slot
     */
    void signal_complete(std::function<void()> slot){
        slot();
    }

    void Complete_notify(){
        emit(Complete_cfg);
    }
};


#endif //WL1_F411CEU6_TELE_JOYSTICK_HPP
