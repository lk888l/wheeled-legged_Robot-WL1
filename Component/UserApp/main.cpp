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
#include <cmath>
#include "format"
/// cpp etl include
//#include "etl/memory.h"
//#include "etl/pool.h"
#include "etl/format.h"
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
///cpp User library include
#include "TB6612.h"
#include "HallEncoder.h"
#include "Servo.hpp"
#include "MPU6050.h"
#include "LkUart.hpp"
#include "CtrlAlgorithm/LQR.hpp"
#include "TaskReactor.hpp"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"



/* My variables define BEGIN */
LkUart<> Uart1(&huart1);
volatile bool isShowIMUData;
volatile bool isShowMotorRAM;
/* My variables define END */
// freeRTOS variably define
TaskHandle_t Handle_LEDBlinkFunc = nullptr;
TaskHandle_t Handle_ServoControlFunc = nullptr;
TaskHandle_t Handle_MotionControlFunc = nullptr;


/*---------------------  define task function begin  ---------------------*/

TaskFunction_t LEDBlinkFunc(){
    TaskReactor t1;
    Uart1.Start_DMAIT_Receive();
    t1.connect(&Uart1,&LkUart<>::signal_RxComplete,[](etl::string<128> &rxmes){
        Uart1.print("receive: {}\n",rxmes);
        if(rxmes.starts_with("servo")){
            uint16_t angle = 0, speed = 0;
            const char* first = rxmes.data() + 6;
            const char* last = rxmes.data() + rxmes.length();
            auto res1 = std::from_chars(first, last, angle);
            if (res1.ec == std::errc{}) {
                // 3. 跳过空格解析第二个数字 (速度)
                auto res2 = std::from_chars(res1.ptr + 1, last, speed);
                if (res2.ec == std::errc{}) {
                    // 4. 封装 32 位通知值并发送
                    uint32_t notifyValue = ((angle & 0xFF) << 20) | (speed & 0xFFF);
                    xTaskNotify(Handle_ServoControlFunc, notifyValue, eSetValueWithOverwrite);
                }
            }
        } else if(rxmes.starts_with("showimuoff")){
            isShowIMUData = false;
        } else if (rxmes.starts_with("showimu")){
            isShowIMUData = true;
        } else if(rxmes.starts_with("showramoff")){
            isShowMotorRAM = false;
        } else if(rxmes.starts_with("showram")){
            isShowMotorRAM = true;
        }
    });

    t1.taskLoop(pdMS_TO_TICKS(500),[](){
//        Uart1.print("hello{}\n",123);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    });
    for(;;){

    }
}

/*---------------------  LQR task function begin  ---------------------*/
TaskFunction_t MotionControlFunc(){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 5;   //(ms)
    ///  c++ variable
    MPU6050 IMU_Main(&hi2c1,{MPU6050::GyroRange_t::G1000,MPU6050::AccRange_t::A4,static_cast<uint16_t>(MPU6050::ms_toHZ(xFrequency)),{0,0,0}});
    MPU6050::EulerAngle MAngle;
//    double IMU_Acc[3];
    double IMU_Gyro[3];
    LQR Ctrl_cal(std::forward<double[4]>({-17.315571f, -3.766466f, 1.195229f, 2.977176f}));
    HallEncoder Enc_Left(&htim2,HallEncoder::InitConfig_t{7, 50, 4, xFrequency});
    HallEncoder Enc_Right(&htim3,HallEncoder::InitConfig_t{7, 50, 4, xFrequency});
    TB6612 TB6_wheel(TB6612::InitConfig_t{.Htim =&htim4,.AChannel = TIM_CHANNEL_3,.BChannel = TIM_CHANNEL_4,
                        .A1GPIO_Port = AIN1_GPIO_Port,.A1GPIO_Pin = AIN1_Pin,.A2GPIO_Port =  AIN2_GPIO_Port,.A2GPIO_Pin = AIN2_Pin,
                        .B1GPIO_Port = BIN1_GPIO_Port,.B1GPIO_Pin = BIN1_Pin,.B2GPIO_Port =  BIN2_GPIO_Port,.B2GPIO_Pin = BIN2_Pin});
    TB6_wheel.Init();
    TB6_wheel.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::A), TB6612::Direction::Negative);
    TB6_wheel.setA_DeadZone(70);TB6_wheel.setB_DeadZone(70);
    //IMU --> MPU6050Init
    IMU_Main.setGyroOffset(2.5,0.7,0.9);
//    TB6_wheel.setAVel_raw(-700);
//    TB6_wheel.setBVel_raw(-700);
    if(IMU_Main.Init()){
        Uart1.print("MPU: success\n");
    }
    else{
        Uart1.print("MPU: fail\n");
    }
    Enc_Left.clearCounter();
    Enc_Right.clearCounter();
    double LQRPos_Left{};
    double LQRPos_Right{};
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro);
//        SendDataToUART1("%lf,%lf,%lf\n",IMU_Gyro[0],IMU_Gyro[1],IMU_Gyro[2]);
        if(isShowIMUData) {
            Uart1.print("{},{},{}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
        }
        double LQR_Angle = MPU6050::DegTorad(MAngle.Pitch + 20.7);
        double LQR_Gyro = MPU6050::DegTorad(IMU_Gyro[2]);
        double RPM_Left = Enc_Left.getRPM();
        double RPM_Right = Enc_Right.getRPM();
        double LQRVel_Left = HallEncoder::Rpm_ToMS(LQR::WheelRadius,RPM_Left) / 60;
        double LQRVel_Right = HallEncoder::Rpm_ToMS(LQR::WheelRadius,RPM_Right) / 60;
        LQRPos_Left += LQRVel_Left*0.005;
        LQRPos_Right += LQRVel_Right*0.005;
//        LQRPos_Left = HallEncoder::Rpm_ToMS(LQR::WheelRadius,HallEncoder::Cnt_toTurnNum(Enc_Left,Enc_Left.getAccumCnt()));
        LQRPos_Left = TB6612::clamp(LQRPos_Left,0.5,-0.5);
//        LQRPos_Right = HallEncoder::Rpm_ToMS(LQR::WheelRadius,HallEncoder::Cnt_toTurnNum(Enc_Right,Enc_Right.getAccumCnt()));
        LQRPos_Right = TB6612::clamp(LQRPos_Right,0.5,-0.5);
        double NeededU_Left = Ctrl_cal.Calculate_LQR(LQR_Angle,LQR_Gyro,LQRPos_Left,LQRVel_Left);
        double NeededU_Right = Ctrl_cal.Calculate_LQR(LQR_Angle,LQR_Gyro,LQRPos_Right,LQRVel_Right);
        int NeededPWM_Left = std::round(NeededU_Left * LQR::TorqueToPWM_Coefficient);
        int NeededPWM_Right = std::round(NeededU_Right * LQR::TorqueToPWM_Coefficient);
        NeededPWM_Left = TB6612::clamp(NeededPWM_Left,1000,-1000);
        NeededPWM_Right = TB6612::clamp(NeededPWM_Right,1000,-1000);
//        SendDataToUART1("%d\n",NeededPWM_Right);
        if(isShowMotorRAM){
            Uart1.print("A: {}\tB: {}\n",RPM_Right,RPM_Left);
        }
//        TB6_wheel.setAVel_raw(NeededPWM_Right);
//        TB6_wheel.setBVel_raw(NeededPWM_Left);
    }
}

/*---------------------  Servo task function begin  ---------------------*/

TaskFunction_t ServoControlFunc(){
    Servo Ser_Lift(&htim9,TIM_CHANNEL_1,500,2500,180.0);
    Servo Ser_Right(&htim9,TIM_CHANNEL_2,
                    Servo::PhysicalToPulse(176.0f),
                    Servo::PhysicalToPulse(4.0f),
                    180.0);
    Ser_Lift.Init();
    Ser_Right.Init();
    uint32_t notifiedValue = 0;
    uint16_t targetAngle,moveSpeed;
    while(1){
        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY ) == pdTRUE){
            targetAngle = (uint16_t)((notifiedValue >> 20) & 0xFF); // 取高 8 位
            moveSpeed = (uint16_t)(notifiedValue & 0xFFF);     // 取低 16 位
            Uart1.print("servoTask: {}, {}\n",targetAngle,moveSpeed);
            if(moveSpeed == 0){
                Ser_Lift.setAngle_Immediate(static_cast<float>(targetAngle));
                Ser_Right.setAngle_Immediate(static_cast<float>(targetAngle));
            } else {
                Ser_Lift.setAngle_Smooth(static_cast<float>(targetAngle),static_cast<float>(moveSpeed));
                Ser_Right.setAngle_Smooth(static_cast<float>(targetAngle),static_cast<float>(moveSpeed));
            }
        }
    }
}

void CPP_Main()
{

    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)LEDBlinkFunc,
                           (const char*)"LEDBlink",
                           (uint16_t)1024,
                           (void*)NULL,
                           (UBaseType_t)28,
                           (TaskHandle_t*)&Handle_LEDBlinkFunc);
    xReturn |= xTaskCreate((TaskFunction_t)ServoControlFunc,
                          (const char*)"ServoControl",
                          (uint16_t)256,
                          (void*)NULL,
                          (UBaseType_t)28,
                          (TaskHandle_t*)&Handle_ServoControlFunc);
    xReturn |= xTaskCreate((TaskFunction_t)MotionControlFunc,
                           (const char*)"MotionControl",
                           (uint16_t)2500,
                           (void*)NULL,
                           (UBaseType_t)29,
                           (TaskHandle_t*)&Handle_MotionControlFunc);

    if(pdPASS == xReturn){
//        SendDataToUART1("success\n");
        Uart1.print("CPPMain: success\n");
    }
    else {
//        SendDataToUART1("fail");
        Uart1.print("CPPMain: fail\n");
    }
}

