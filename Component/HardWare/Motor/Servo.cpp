/********************************************************************************
  * @file           : Servo.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-3-25
  *******************************************************************************/


#include "Servo.hpp"

// 定时器回调函数（静态 C 风格函数，转发给类成员）
void ServoTimerCallback(TimerHandle_t xTimer) {
    Servo* s = (Servo*)pvTimerGetTimerID(xTimer);
    s->updateSmoothing();
}

Servo::Servo(TIM_HandleTypeDef *htim, uint32_t channel, int32_t minPulseUs, int32_t maxPulseUs, float maxAngle, float angle)
        : HTim(htim), TimChannel(channel)
        , MinPulse(minPulseUs), MaxPulse(maxPulseUs), MaxAngle(maxAngle)
{
    // 创建软件定时器，周期为 20ms
    xTimer = xTimerCreate("ServoTimer", pdMS_TO_TICKS(UPDATE_PERIOD_MS),
                      pdTRUE, (void*)this, ServoTimerCallback);
    K_Pulse = (float)(maxPulseUs - minPulseUs) / maxAngle;
    B_Pulse = (float)minPulseUs;
    setAngle_Immediate(angle);

}

void Servo::setPWM_FromAngle(float angle) {
    if (angle < 0) angle = 0;
    else if (angle > MaxAngle) angle = MaxAngle;
    // 映射：角度 -> 脉冲宽度
    // 公式：脉宽 = 最小脉宽 + (当前角度 / 最大角度) * (最大脉宽 - 最小脉宽)
//    uint32_t pulse = MinPulse + (uint32_t)((angle / MaxAngle) * (MaxPulse - MinPulse));
    uint32_t pulse = (uint32_t)(angle * K_Pulse + B_Pulse);
    __HAL_TIM_SET_COMPARE(HTim, TimChannel, pulse);
}


bool Servo::Init() {
    return static_cast<bool>(HAL_TIM_PWM_Start(HTim, TimChannel));
}

void Servo::stop() {
    HAL_TIM_PWM_Stop(HTim, TimChannel);
}

bool Servo::setAngle_Immediate(float angle) {
    xTimerStop(xTimer, 0);          //stop soft timer to immediate execution
    setPWM_FromAngle(angle);
    CurrentAngle = TargetAngle = angle;
    return true;
}

void Servo::setAngle_Smooth(float targetAngle, float speed) {
    TargetAngle = targetAngle;
    StepSize = speed * (UPDATE_PERIOD_MS / 1000.0f);
    xTimerStart(xTimer, 0);     // start soft timer
}

float Servo::getCurrentAngle() const {
    return CurrentAngle;
}

void Servo::updateSmoothing() {
    float diff = TargetAngle - CurrentAngle;
    if (this->abs(diff) <= StepSize) {
        CurrentAngle = TargetAngle;
        xTimerStop(xTimer, 0); // 到达目的地，停掉定时器节省资源
    }
    else {
        // 向目标方向迈进一步
        if (diff > 0) CurrentAngle += StepSize;
        else CurrentAngle -= StepSize;
    }
    setPWM_FromAngle(CurrentAngle);
}

int32_t Servo::PhysicalToPulse(float physicalAngle, float baseMinPulse, float baseMaxPulse, float baseMaxAngle) {
    return (int32_t)(baseMinPulse + (physicalAngle / baseMaxAngle) * (baseMaxPulse - baseMinPulse));
}









