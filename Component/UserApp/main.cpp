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
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "i2c.h"
#include "TB6612.h"
#include "MPU6050.h"
#include "basicvqf.hpp"
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
    std::unique_ptr<TB6612> pTB6_wheel;
    std::unique_ptr<MPU6050> pM6050_Imu;
    std::unique_ptr<VQF> basicVqf;
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
    pTB6_wheel->Init();
    pTB6_wheel->setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B), TB6612::Direction::Negative);
    pTB6_wheel->setAVel_raw(500);
    pTB6_wheel->setBVel_raw(500);

    if(pM6050_Imu->Init()){
        SendDataToUART1("MPU: success\n");
    }
    else{
        SendDataToUART1("MPU: fail\n");
    }
    for(;;){
        double xg,yg,zg,xa,ya,za;
        vTaskDelay(7);
        pM6050_Imu->getGyro(xg,yg,zg);
        pM6050_Imu->getAccel(xa,ya,za);
//        SendDataToUART1("xg: %lf\tyg: %lf\tzg: %lf\txa: %lf\tya: %lf\tza: %lf\t\n",xg,yg,zg,xa,ya,za);
        xg = (xg + 2.5) * PI / 180;
        yg = (yg + 0.7) * PI / 180;
        zg = (zg + 0.9) * PI / 180;
        xa = (xa) * 9.81;
        ya = (ya) * 9.81;
        za = (za) * 9.81;
        vqf_real_t gyr[3] = {xg,yg,zg};
        vqf_real_t acc[3] = {xa,ya,za};
        vqf_real_t quat[4] = {0, 0, 0, 0}; // output array for quaternion
        vqf_real_t bias[3];
        basicVqf->update(gyr,acc);
        basicVqf->getQuat6D(quat);
        auto q0 = quat[0];
        auto q1 = quat[1];
        auto q2 = quat[2];
        auto q3 = quat[3];
        auto roll = atan2(2 * (q0 * q1 + q2 * q3), q0*q0 - q1*q1 - q2*q2 + q3*q3)* 57.295773;
        auto pitch = -asin(2 * (q1 * q3 - q0 * q2))*57.295773;
        auto yaw = atan2(2 * (q0 * q3 + q1 * q2), q0*q0 + q1*q1 - q2*q2 - q3*q3)*57.295773;
//        SendDataToUART1("%f,%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3]);
        SendDataToUART1("%f,%f,%f\n",roll,pitch,yaw);
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
                           (uint16_t)1000,
                           (void*)NULL,
                           (UBaseType_t)28,
                           (TaskHandle_t*)&Handle_MotionControlFunc);
    if(pdPASS == xReturn){
//        SendDataToUART1("success\n");
    }
    else {
        SendDataToUART1("fail");
    }
}