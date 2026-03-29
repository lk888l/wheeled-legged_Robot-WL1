/********************************************************************************
  * @file           : Servo.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-25
  *******************************************************************************/

#ifdef __GNUC__
#pragma once
#endif //__GNUC__
#ifndef __SERVO_HPP
#define __SERVO_HPP

//stm32-hal library include
#include "main.h"
//freeRTOS library include
#include "freertos.h"
#include "timers.h"     // freeRTOS soft timer


class Servo {
public:
    /**
     * @brief 舵机类构造函数
     * @param htim      定时器句柄指针 (例如: &htim1)
     * @param channel   定时器通道 (例如: TIM_CHANNEL_1)
     * @param minPulseUs 0度对应的脉宽(微秒)，通常为 500
     * @param maxPulseUs 180度对应的脉宽(微秒)，通常为 2500
     * @param maxAngle  舵机的最大角度，通常为 180.0或270.0
     * @param angle     Init angle
     */
    Servo(TIM_HandleTypeDef* htim, uint32_t channel,int32_t minPulseUs, int32_t maxPulseUs,float maxAngle, float angle = 0.0f);
    ~Servo(){stop();}

private:
    TIM_HandleTypeDef* HTim;
    uint32_t TimChannel;

    uint32_t MinPulse;
    uint32_t MaxPulse;
    float MaxAngle;
    float CurrentAngle;
    float TargetAngle{};        // 用户期望到达的角度
    float StepSize{};           // 每一帧移动的步长
    TimerHandle_t xTimer;     // FreeRTOS 软件定时器句柄
    static const uint32_t UPDATE_PERIOD_MS = 10; // 20ms 更新一次，匹配 50Hz PWM

    float K_Pulse;
    float B_Pulse;

    inline void setPWM_FromAngle(float angle);

public:
    bool Init();
    void stop();

    /**
     * @brief immediate execution.
     * @param angle
     * @return
     */
    bool setAngle_Immediate(float angle);

    /**
     * @brief Execute smoothly at the preset speed.
     * @param targetAngle
     * @param speed
     */
    void setAngle_Smooth(float targetAngle, float speed);
    /**
     * @brief
     * @return
     */
    float getCurrentAngle() const;
    /**
     * @brief freeRTOS soft timer callback.
     */
    void updateSmoothing();

    static int32_t PhysicalToPulse(float physicalAngle, float baseMinPulse = 500.0f, float baseMaxPulse = 2500.0f, float baseMaxAngle = 180.0f);

    template<typename type>
    static inline type abs(type v){
        return v < 0 ? -v : v;
    }
};


#endif //__SERVO_HPP
