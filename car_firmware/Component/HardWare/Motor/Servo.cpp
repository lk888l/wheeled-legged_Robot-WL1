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
    // 创建软件定时器，周期为 10ms
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
    if (xTimer == nullptr || HAL_TIM_PWM_Start(HTim, TimChannel) != HAL_OK) {
        return false;
    }
    // Keep the lightweight 10 ms updater periodic. New targets only replace
    // TargetAngle; they never restart the timer and therefore cannot postpone it.
    return xTimerStart(xTimer, 0) == pdPASS;
}

void Servo::stop() {
    if (xTimer != nullptr) {
        xTimerStop(xTimer, 0);
    }
    HAL_TIM_PWM_Stop(HTim, TimChannel);
}

bool Servo::setAngle_Immediate(float angle) {
    if(Limit_Max_Angle!=0 && angle>Limit_Max_Angle)   angle = Limit_Max_Angle;
    else if(angle<Limit_Min_Angle)  angle = Limit_Min_Angle;
    taskENTER_CRITICAL();
    CurrentAngle = TargetAngle = angle;
    StepSize = 0.0F;
    taskEXIT_CRITICAL();
    setPWM_FromAngle(angle);
    return true;
}

void Servo::setAngle_Smooth(float targetAngle, float speed) {
    if(Limit_Max_Angle!=0 && targetAngle>Limit_Max_Angle)   targetAngle = Limit_Max_Angle;
    else if(targetAngle<Limit_Min_Angle)  targetAngle = Limit_Min_Angle;
    if(speed <= 0 || xTimer == nullptr){
        setAngle_Immediate(targetAngle);
        return;
    }
    taskENTER_CRITICAL();
    TargetAngle = targetAngle;
    StepSize = speed * (UPDATE_PERIOD_MS / 1000.0f);
    taskEXIT_CRITICAL();
}

float Servo::getCurrentAngle() const {
    taskENTER_CRITICAL();
    const float angle = CurrentAngle;
    taskEXIT_CRITICAL();
    return angle;
}

bool Servo::isCommandComplete(float tolerance_degrees) const {
    if (tolerance_degrees < 0.0F) tolerance_degrees = 0.0F;
    taskENTER_CRITICAL();
    const bool complete = this->abs(TargetAngle - CurrentAngle) <= tolerance_degrees;
    taskEXIT_CRITICAL();
    return complete;
}

void Servo::updateSmoothing() {
    taskENTER_CRITICAL();
    const float diff = TargetAngle - CurrentAngle;
    if (this->abs(diff) <= StepSize) {
        CurrentAngle = TargetAngle;
    }
    else {
        // 向目标方向迈进一步
        if (diff > 0) CurrentAngle += StepSize;
        else CurrentAngle -= StepSize;
    }
    const float angle = CurrentAngle;
    taskEXIT_CRITICAL();
    setPWM_FromAngle(angle);
}

int32_t Servo::PhysicalToPulse(float physicalAngle, float baseMinPulse, float baseMaxPulse, float baseMaxAngle) {
    return (int32_t)(baseMinPulse + (physicalAngle / baseMaxAngle) * (baseMaxPulse - baseMinPulse));
}









