/********************************************************************************
  * @file           : main.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 12/17/2025
  *******************************************************************************/


/**
 * Created by kk on 2025/7/17.
*/

/// cpp standard library include
#include <memory>
#include <cmath>
#include <print>
#include <format>
/// cpp etl include
//#include "etl/memory.h"
//#include "etl/pool.h"
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "i2c.h"
///cpp User library include
#include "TB6612.h"
#include "HallEncoder.h"
#include "MPU6050.h"
#include "LkUart.hpp"
#include "CtrlAlgorithm/LQR.hpp"



/* My variables BEGIN */

/* My variables END */

/*---------------------  define task function begin  ---------------------*/

TaskFunction_t LEDBlinkFunc(){
    while(1) {
        vTaskDelay(200);
//        SendDataToUART1("hello\n");
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}
/*---------------------  define task function begin  ---------------------*/



TaskFunction_t MotionControlFunc(){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    ///  c++ variable
    MPU6050 IMU_Main(&hi2c1);
    MPU6050::EulerAngle MAngle;
//    double IMU_Acc[3];
    double IMU_Gyro[3];
    LQR Ctrl_cal(std::forward<double[4]>({-4.7731,-0.9900,1.0000,1.5385}));
    HallEncoder Enc_Left(&htim2,HallEncoder::InitConfig_t{7, 50, 4, 10});
    HallEncoder Enc_Right(&htim3,HallEncoder::InitConfig_t{7, 50, 4, 10});
    TB6612 TB6_wheel(TB6612::InitConfig_t{.Htim =&htim4,.AChannel = TIM_CHANNEL_3,.BChannel = TIM_CHANNEL_4,
                        .A1GPIO_Port =  AIN1_GPIO_Port,.A1GPIO_Pin =   AIN1_Pin,.A2GPIO_Port =  AIN2_GPIO_Port,.A2GPIO_Pin =   AIN2_Pin,
                        .B1GPIO_Port =  BIN1_GPIO_Port,.B1GPIO_Pin =   BIN1_Pin,.B2GPIO_Port =  BIN2_GPIO_Port,.B2GPIO_Pin =   BIN2_Pin});
    TB6_wheel.Init();
    TB6_wheel.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B), TB6612::Direction::Negative);
    //IMU --> MPU6050Init
    IMU_Main.setGyroOffset(2.5,0.7,0.9);
    if(IMU_Main.Init()){
        SendDataToUART1("MPU: success\n");
    }
    else{
        SendDataToUART1("MPU: fail\n");
    }
    Enc_Left.clearCounter();
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro);
//        SendDataToUART1("%lf,%lf,%lf\n",MAngle.Roll,MAngle.Pitch,MAngle.Yaw);
        double LQR_Angle = MPU6050::DegTorad(MAngle.Pitch + 20);
        double LQR_Gyro = MPU6050::DegTorad(IMU_Gyro[2]);
        double LQR_Vel = HallEncoder::Rpm_ToMS(LQR::WheelRadius,Enc_Left.getRPM()) / 60;
        double LQR_Pos = HallEncoder::Rpm_ToMS(LQR::WheelRadius,HallEncoder::Cnt_toTurnNum(Enc_Left,Enc_Left.getAccumCnt()));
        double Needed_U = Ctrl_cal.Calculate_LQR(LQR_Angle,LQR_Gyro,LQR_Pos,LQR_Vel);
        int Needed_PWM = std::round(Needed_U * LQR::TorqueToPWM_Coefficient);
        Needed_PWM = TB6612::clamp(Needed_PWM,1000,-1000);
        TB6_wheel.setAVel_raw(-Needed_PWM);
        TB6_wheel.setBVel_raw(-Needed_PWM);
    }
}

void CPP_Main()
{
    TaskHandle_t Handle_LEDBlinkFunc = nullptr;
    TaskHandle_t Handle_MotionControlFunc = nullptr;
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)LEDBlinkFunc,
                          (const char*)"LEDBlink",
                          (uint16_t)128,
                          (void*)NULL,
                          (UBaseType_t)28,
                          (TaskHandle_t*)&Handle_LEDBlinkFunc);
    xReturn |= xTaskCreate((TaskFunction_t)MotionControlFunc,
                           (const char*)"MotionControl",
                           (uint16_t)2200,
                           (void*)NULL,
                           (UBaseType_t)29,
                           (TaskHandle_t*)&Handle_MotionControlFunc);
    if(pdPASS == xReturn){
//        SendDataToUART1("success\n");
    }
    else {
        SendDataToUART1("fail");
    }
}