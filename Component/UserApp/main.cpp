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
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "i2c.h"
#include "TB6612.h"
#include "MPU6050.h"
#include "vqf.hpp"

#define PI 3.14159265358979323846
#define SQRT3 5.663806

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
    //  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    //  c++ variable
    std::unique_ptr<TB6612> pTB6_wheel;
    std::unique_ptr<MPU6050> pM6050_Imu;
    std::unique_ptr<VQF> basicVqf;
    double GyroValue[3]{};
    double AccValue[3]{};
    MPU6050::EulerAngle MAngle;
    vqf_real_t quat[4]{}; // output array for quaternion
    {
        TB6612::InitConfig_t TB6_temcfg = {
                                .Htim =         &htim4,
                                .AChannel =     TIM_CHANNEL_3,
                                .BChannel =     TIM_CHANNEL_4,
                                .A1GPIO_Port =  AIN1_GPIO_Port,
                                .A1GPIO_Pin =   AIN1_Pin,
                                .A2GPIO_Port =  AIN2_GPIO_Port,
                                .A2GPIO_Pin =   AIN2_Pin,
                                .B1GPIO_Port =  BIN1_GPIO_Port,
                                .B1GPIO_Pin =   BIN1_Pin,
                                .B2GPIO_Port =  BIN2_GPIO_Port,
                                .B2GPIO_Pin =   BIN2_Pin,
                        };
        pTB6_wheel = std::make_unique<TB6612>(TB6_temcfg);
        pM6050_Imu = std::make_unique<MPU6050>(&hi2c1);
        basicVqf = std::make_unique<VQF>(0.01,0.005);
    }
//    pTB6_wheel->Init();
//    pTB6_wheel->setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B), TB6612::Direction::Negative);
//    pTB6_wheel->setAVel_raw(500);
//    pTB6_wheel->setBVel_raw(500);
    pM6050_Imu->setGyroOffset(2.5,0.7,0.9);
    if(pM6050_Imu->Init()){
        SendDataToUART1("MPU: success\n");
    }
    else{
        SendDataToUART1("MPU: fail\n");
    }
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        pM6050_Imu->getGyro(GyroValue);
        pM6050_Imu->getAccel(AccValue);
//        SendDataToUART1("xg: %lf\tyg: %lf\tzg: %lf\txa: %lf\tya: %lf\tza: %lf\t\n",GyroValue[0],GyroValue[1],GyroValue[2],AccValue[0],AccValue[1],AccValue[2]);
        MPU6050::DegTorad(GyroValue);
        MPU6050::GToMS2(AccValue);
        basicVqf->update(GyroValue,AccValue);
        basicVqf->getQuat6D(quat);
        MPU6050::QuatToEuler(quat,MAngle);
        SendDataToUART1("%lf,%lf,%lf\n",MAngle.Roll,MAngle.Pitch,MAngle.Yaw);

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
                           (uint16_t)2000,
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