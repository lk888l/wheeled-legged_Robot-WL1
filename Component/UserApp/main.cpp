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
#include <atomic>
/// cpp etl include
//#include "etl/memory.h"
//#include "etl/pool.h"
#include "etl/unordered_map.h"
#include "etl/format.h"
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
///cpp User library include
#include "TB6612.h"
#include "HallEncoder.h"
#include "Servo.hpp"
#include "MPU6050.h"
#include "NRF24L01P.hpp"
#include "LkUart.hpp"
#include "TaskReactor.hpp"
#include "CtrlAlgorithm/LQR.hpp"
#include "CtrlAlgorithm/PID.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"



/* My variables define BEGIN */
LkUart<> Uart1(&huart1);
volatile bool isShowIMUData;
volatile bool isShowMotorRAM;
/// PID
// angle-pid
volatile float Angle_kp{70.0f};
volatile float Angle_ki{};
volatile float Angle_kd{51.0f};
volatile float Angle_bias{12.6f};    //静态角度偏差（向前则减小）
// velocity-pid
volatile float Velocity_kp{0.05f};
volatile float Velocity_ki{0.008f};
volatile float Velocity_kd{};
volatile float Velocity_Target{};
//differ-pid
volatile float Differ_kp{2.0f};
volatile float Differ_ki{0.001};
volatile float Differ_kd{};
volatile float Differ_Target{};
//adapt_y pid
volatile float Adapt_y_kp{};
volatile float Adapt_y_ki{-0.4};
/// real Angle value
volatile float EAngle_print[3]{};
/// Leg height
volatile float Left_Legheight{};
volatile float Right_Legheight{};
volatile float Target_height{44.5f};
volatile float Roll_Target{};
volatile float LWheel_x{};
volatile float RWheel_x{};
/// telecontrol
NRF24L01P nRF(&hspi2,GPIOA,GPIO_PIN_4,GPIOB,GPIO_PIN_12,GPIOA,GPIO_PIN_12);
volatile bool isNRF_print{};
const volatile float* NRF_print[4]{reinterpret_cast<const volatile float *>(&EAngle_print[0]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[1]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[2]),
                                   reinterpret_cast<const volatile float *>(&Angle_kp)};
/* My variables define END */

// freeRTOS variably define
TaskHandle_t Handle_LEDBlinkFunc = nullptr;
TaskHandle_t Handle_ServoControlFunc = nullptr;
TaskHandle_t Handle_MotionControlFunc = nullptr;


/*---------------------  define task function begin  ---------------------*/
using CommandHandler = std::function<void(etl::string_view)>;
TaskFunction_t LEDBlinkFunc(){
    TaskReactor t1;
    TaskReactor::strCMD_t Uart_CMD;
    uint8_t NRF_Tx_Num[NRF24L01P::PACKET_WIDTH]{};
    uint8_t NRF_Rx_Num[NRF24L01P::PACKET_WIDTH]{};
    etl::string_view NRF_TxStr = "NRF: 1\n";
    etl::string_view NRF_RxStr{};
    etl::unordered_map<etl::string_view,CommandHandler,25,57> cmdMap = {
//            {"servo", [](etl::string_view args) {
//                uint16_t angle{},speed{};
//                if(TaskReactor::parseStrArg(args,angle) && TaskReactor::parseStrArg(args,speed)){
//                    uint32_t notifyValue = ((angle & 0xFF) << 20) | (speed & 0xFFF);
//                    xTaskNotify(Handle_ServoControlFunc, notifyValue, eSetValueWithOverwrite);
//                }
//                else{
//                    Uart1.print("Command \"servo\": Useless parameters\n");
//                }
//            }},
            {"motor",[](etl::string_view args){
                uint16_t vL = 0, vR = 0;
                if(TaskReactor::parseStrArg(args,vL) && TaskReactor::parseStrArg(args,vR)){
                    Uart1.print("motor: {}\t{}\n",vL,vR);
                    uint32_t notifyValue = (vL << 16) | (vR & 0xFFFF);
                    xTaskNotify(Handle_MotionControlFunc, notifyValue, eSetValueWithOverwrite);
                }
                else{
                    Uart1.print("Command \"servo\": Useless parameters\n");
                }
            }},
            {"showimu",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    if(args[1] == 'y'){isShowIMUData = true;}
                    else if(args[1] == 'n'){isShowIMUData = false;}
                }
            }},
            {"showrpm",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    if(args[1] == 'y'){isShowMotorRAM = true;}
                    else if(args[1] == 'n'){isShowMotorRAM = false;}
                }
            }},
            {"anglepid",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    float value{};
                    if(args[1] == 'p'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Angle_kp = value;
                        }
                    }
                    else if(args[1] == 'i'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Angle_ki = value;
                        }
                    }
                    else if(args[1] == 'd'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Angle_kd = value;
                        }
                    }
                }
            }},
            {"velocitypid",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    float value{};
                    if(args[1] == 'p'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Velocity_kp = value;
                        }
                    }
                    else if(args[1] == 'i'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Velocity_ki = value;
                        }
                    }
                    else if(args[1] == 'd'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Velocity_kd = value;
                        }
                    }
                }

            }},
            {"differpid",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    float value{};
                    if(args[1] == 'p'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Differ_kp = value;
                        }
                    }
                    else if(args[1] == 'i'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Differ_ki = value;
                        }
                    }
                    else if(args[1] == 'd'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Differ_kd = value;
                        }
                    }
                }
            }},
            {"rollpid",[](etl::string_view args){
                if(args.size() >= 2 && args[0] == '-'){
                    float value{};
                    if(args[1] == 'p'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Adapt_y_ki = value;
                        }
                    }
                    else if(args[1] == 'i'){
                        args.remove_prefix(3);
                        if(TaskReactor::parseStrArg(args,value)){
                            Adapt_y_ki = value;
                        }
                    }
                }
            }},
            {"nrfsend",[&NRF_Tx_Num](etl::string_view args){
                NRF24L01P::str_touint8(args, NRF_Tx_Num);
                nRF.send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH);
            }},
            {"nrfshow",[](etl::string_view args){
                if(args.size() >= 3 && args[0] == '-'){
                    uint8_t value{};
                    if(args[1] == 'm' && args[2] == 'r'){
                        args.remove_prefix(4);
                        if(TaskReactor::parseStrArg(args,value)){
                            if(value>3) value=3;
                            NRF_print[value] = reinterpret_cast<const volatile float *>(&EAngle_print[0]);
                            isNRF_print = true;
                        }
                    }
                    else if(args[1] == 'm' && args[2] == 'p'){
                        args.remove_prefix(4);
                        if(TaskReactor::parseStrArg(args,value)){
                            if(value>3) value=3;
                            NRF_print[value] = reinterpret_cast<const volatile float *>(&EAngle_print[1]);
                            isNRF_print = true;
                        }
                    }
                    else if(args[1] == 'm' && args[2] == 'y'){
                        args.remove_prefix(4);
                        if(TaskReactor::parseStrArg(args,value)){
                            if(value>3) value=3;
                            NRF_print[value] = reinterpret_cast<const volatile float *>(&EAngle_print[2]);
                            isNRF_print = true;
                        }
                    }
                    else if(args[1] == 'n' && args[2] == 'n'){
                        isNRF_print = false;
                    }
                }
            }},
            {"legheight",[](etl::string_view args){
                float height{}, result_x{}, result_deg{};
                if(TaskReactor::parseStrArg(args,height)){
                    result_deg = LegKinematics::getMotorAngleForHeight(height,&result_x);
                    float bais_p = ((-0.000155f * height + 0.03882f) * height + -3.001f) * height + 83.25f;
                    Uart1.print("Servo angel: {:07.3f} {:07.3f} {:07.3f}\n",result_deg,result_x,bais_p);
                    Target_height = height;
                }
            }},
            {"target_roll",[](etl::string_view args){
                float tar_roll{};
                if(TaskReactor::parseStrArg(args,tar_roll)){
                    Roll_Target = tar_roll;
                }
            }},
            {"target_roll",[](etl::string_view args){
                float tar_roll{};
                if(TaskReactor::parseStrArg(args,tar_roll)){
                    Roll_Target = tar_roll;
                }
            }},
            {"VandD",[](etl::string_view args){
                float target_v{},target_d{};
                if(TaskReactor::parseStrArg(args,target_d) && TaskReactor::parseStrArg(args,target_v)){
                    Differ_Target = target_d;
                    Velocity_Target = target_v;
                }
            }},
            {"anglebias",[](etl::string_view args){
                float bias{};
                if(TaskReactor::parseStrArg(args,bias)){
                    Angle_bias = bias;
                }
            }},
    };
    etl::queue<etl::string<32>,4> CMD_que;
    /// Init NRF
    nRF.Init();
    nRF.start_RxMode();
    /// connect Uart1
    Uart1.Start_DMAIT_Receive();
    t1.connect(&Uart1,&LkUart<>::signal_RxComplete,[&Uart_CMD,&cmdMap,&CMD_que](etl::string<128> &rxmes){
//        Uart1.print("receive: {}\n",rxmes);
//        if(TaskReactor::parseStrCMD(rxmes,Uart_CMD)){
//            auto it = cmdMap.find(Uart_CMD.command);
//            if (it != cmdMap.end()) {
//                it->second(Uart_CMD.args); // 执行对应的 Lambda 或函数
//            } else {
//                Uart1.print("Unknown command!\n");
//            }
//        }
        if(rxmes.size()>32) {CMD_que.push(rxmes.substr(0, 32));}
        else    {CMD_que.push(rxmes);}
//        rxmes.insert(0, "Roger：");
//        NRF24L01P::str_touint8(rxmes, NRF_Tx_Num);
//        nRF.send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH);
    });
    /// connect NRF
    t1.connect(&nRF,&NRF24L01P::signal_IRQEvent,[&NRF_Rx_Num,&NRF_RxStr,&CMD_que](NRF24L01P::Status_t &curStatus){
        if(curStatus.RX_DR){
            nRF.tryReceive(NRF_Rx_Num);
            NRF24L01P::uint8_tostr(NRF_RxStr,NRF_Rx_Num);
            if(!CMD_que.full()) CMD_que.push(etl::string<32>(NRF_RxStr));
        }
        if(curStatus.TX_DS){
            Uart1.print("nRF: send success\n");
        }
        if(curStatus.MAX_RT){
            Uart1.print("nRF: send fail\n");
        }
    });
    t1.taskLoop(pdMS_TO_TICKS(100),[&CMD_que,&Uart_CMD,&cmdMap](){
        while(!CMD_que.empty()){
            auto cmd = CMD_que.front();
            CMD_que.pop();
            if(TaskReactor::parseStrCMD(cmd,Uart_CMD)){
                auto it = cmdMap.find(Uart_CMD.command);
                if (it != cmdMap.end()) {
                    it->second(Uart_CMD.args); // 执行对应的 Lambda 或函数
                } else {
                    //Unknown command!
                    Uart1.print("receive: {}\n",cmd);
                }
            }
            else{
                Uart1.print("receive: {}\n",cmd);
            }
        }
    },
[&NRF_Tx_Num,&NRF_TxStr](){
//        Uart1.print("hello{}\n",123);
        if(isNRF_print){
            NRF24L01P::args_touint8s(NRF_Tx_Num,NRF_print);
            nRF.send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH);
        }
//        NRF24L01P::args_touint8s(NRF_Tx_Num,NRF_print);
//        etl::string_view debugargs(reinterpret_cast<const char*>(NRF_Tx_Num),32);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    });
    for(;;){

    }
}

/*---------------------  LQR task function begin  ---------------------*/
TaskFunction_t MotionControlFunc(){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;   //(ms)
    uint32_t notifiedValue_0{};
    ///  c++ variable
    MPU6050 IMU_Main(&hi2c1,{MPU6050::GyroRange_t::G1000,MPU6050::AccRange_t::A4,static_cast<uint16_t>(MPU6050::ms_toHZ(2)),{0,0,0}});
    MPU6050::EulerAngle MAngle;
    //    double IMU_Acc[3];
    double IMU_Gyro[3];
    LQR Ctrl_cal(std::forward<double[4]>({-4.569790f, -4.472503f, -0.000000f, 0.000000f}));
    HallEncoder Enc_Left(&htim2,HallEncoder::InitConfig_t{7, 150, 4, xFrequency});
    HallEncoder Enc_Right(&htim3,HallEncoder::InitConfig_t{7, 150, 4, xFrequency});
    TB6612 TB6_wheel(TB6612::InitConfig_t{.Htim =&htim1,.AChannel = TIM_CHANNEL_1,.BChannel = TIM_CHANNEL_2,
                        .A1GPIO_Port = AIN1_GPIO_Port,.A1GPIO_Pin = AIN1_Pin,.A2GPIO_Port =  AIN2_GPIO_Port,.A2GPIO_Pin = AIN2_Pin,
                        .B1GPIO_Port = BIN1_GPIO_Port,.B1GPIO_Pin = BIN1_Pin,.B2GPIO_Port =  BIN2_GPIO_Port,.B2GPIO_Pin = BIN2_Pin});
    TB6_wheel.Init();
    TB6_wheel.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::A), TB6612::Direction::Negative);
    TB6_wheel.setA_DeadZone(0);TB6_wheel.setB_DeadZone(0);
    //IMU --> MPU6050Init
    IMU_Main.setGyroOffset(2.5,0.7,0.9);
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
        taskENTER_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        static double filtered_RPM_Left = 0;
        static double filtered_RPM_Right = 0;
        IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro);
        if(isShowIMUData) {
            Uart1.print("{:07.3f},{:07.3f},{:07.3f}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
        }
        double LQR_Angle = MPU6050::DegTorad(MAngle.Pitch + 20.0);
        double LQR_Gyro = -MPU6050::DegTorad(IMU_Gyro[1]);
        double RPM_Left_Raw = Enc_Left.getRPM();
        double RPM_Right_Raw = Enc_Right.getRPM();
        filtered_RPM_Left = -RPM_Left_Raw;
        filtered_RPM_Right = -RPM_Right_Raw;
        double LQRVel_Left = HallEncoder::Rpm_ToMS(LQR::WheelRadius,filtered_RPM_Left) / 60;
        double LQRVel_Right = HallEncoder::Rpm_ToMS(LQR::WheelRadius,filtered_RPM_Right) / 60;
//        LQRPos_Left += LQRVel_Left*0.005;
//        LQRPos_Right += LQRVel_Right*0.005;
        LQRPos_Left = -HallEncoder::Rpm_ToMS(LQR::WheelRadius,HallEncoder::Cnt_toTurnNum(Enc_Left,Enc_Left.getAccumCnt()));
        LQRPos_Left = TB6612::clamp(LQRPos_Left,5.0,-5.0);
        LQRPos_Right = -HallEncoder::Rpm_ToMS(LQR::WheelRadius,HallEncoder::Cnt_toTurnNum(Enc_Right,Enc_Right.getAccumCnt()));
        LQRPos_Right = TB6612::clamp(LQRPos_Right,5.0,-5.0);
        double NeededU_Left = Ctrl_cal.Calculate_LQR(LQR_Angle,LQR_Gyro,LQRPos_Left,LQRVel_Left);
        double NeededU_Right = Ctrl_cal.Calculate_LQR(LQR_Angle,LQR_Gyro,LQRPos_Right,LQRVel_Right);
//        int NeededPWM_Left = std::round(NeededU_Left * LQR::TorqueToPWM_Coefficient);
//        int NeededPWM_Right = std::round(NeededU_Right * LQR::TorqueToPWM_Coefficient);
        int NeededPWM_Left = std::round(NeededU_Left * 1200);
        int NeededPWM_Right = std::round(NeededU_Right * 1200);
        NeededPWM_Left = TB6612::clamp(NeededPWM_Left,1000,-1000);
        NeededPWM_Right = TB6612::clamp(NeededPWM_Right,1000,-1000);
        if(isShowMotorRAM){
            Uart1.print("A: {:07.3f}\tB: {:07.3f}\n",RPM_Left_Raw,RPM_Right_Raw);
        }
        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue_0, 0 ) == pdTRUE){
            Uart1.print("Motor output:{}\t{}\n",(notifiedValue_0>>14),(notifiedValue_0 & 0xFFFF));
            TB6_wheel.setBVel_raw(static_cast<int16_t>(notifiedValue_0>>14));
            TB6_wheel.setAVel_raw(static_cast<int16_t>(notifiedValue_0 & 0xFFFF));
        }
//        TB6_wheel.setAVel_raw(-NeededPWM_Right);
//        TB6_wheel.setBVel_raw(-NeededPWM_Left);
        taskEXIT_CRITICAL();
    }
}


/*---------------------  PID task function begin  ---------------------*/
TaskFunction_t MotionControlFunc_PID(){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;   //(ms)
    uint8_t vel_loop_cnt = 0;
    uint32_t notifiedValue_0{};
    ///  c++ variable
    //  PID
    PID Angle_PID(70.0f,0,51.0f,-1000,1000,-100,100);
    PID Velocity_PID(0.04,0.006,0,-10,10,-100,100);
    PID Differ_PID(0,0,0,-200,200,-100,100);
    PID AdaptY_PID(0,0,0,-78,78,-100,100);
    float Differ_RPM, Angle_target{},DifferPWM{};
    //sensor
    MPU6050 IMU_Main(&hi2c1,{MPU6050::GyroRange_t::G1000,MPU6050::AccRange_t::A4,static_cast<uint16_t>(MPU6050::ms_toHZ(xFrequency)),{0,0,0}});
    MPU6050::EulerAngle MAngle;
    double IMU_Gyro[3];
    HallEncoder Enc_Left(&htim2,HallEncoder::InitConfig_t{7, 50, 4, 50});
    HallEncoder Enc_Right(&htim3,HallEncoder::InitConfig_t{7, 50, 4, 50});
    // Motor derive
    TB6612 TB6_wheel(TB6612::InitConfig_t{.Htim =&htim1,.AChannel = TIM_CHANNEL_1,.BChannel = TIM_CHANNEL_2,
            .A1GPIO_Port = AIN1_GPIO_Port,.A1GPIO_Pin = AIN1_Pin,.A2GPIO_Port =  AIN2_GPIO_Port,.A2GPIO_Pin = AIN2_Pin,
            .B1GPIO_Port = BIN1_GPIO_Port,.B1GPIO_Pin = BIN1_Pin,.B2GPIO_Port =  BIN2_GPIO_Port,.B2GPIO_Pin = BIN2_Pin});
    TB6_wheel.Init();
    TB6_wheel.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B), TB6612::Direction::Negative);
    TB6_wheel.setA_DeadZone(50);TB6_wheel.setB_DeadZone(50);
    //IMU --> MPU6050Init
    IMU_Main.setGyroOffset(2.5,0.7,0.9);
    if(IMU_Main.Init()){
        Uart1.print("MPU: success\n");
    }
    else{
        Uart1.print("MPU: fail\n");
    }
    Enc_Left.clearCounter();
    Enc_Right.clearCounter();
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    while(1){
        taskENTER_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // read now Leg height to calculate pitch angle compensate and angle_kp
        float Y_avg = (Left_Legheight + Left_Legheight) / 2.0f;
        Angle_bias = (0.01026f * Y_avg * Y_avg) - (1.258f * Y_avg) + 48.24f;
        Angle_kp = (0.3f*Y_avg) + 56.9;
        // get IMU euler angle
        IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro);
        EAngle_print[0] = static_cast<volatile float>(MAngle.Roll); EAngle_print[1] = static_cast<volatile float>(MAngle.Pitch);EAngle_print[2] = static_cast<volatile float>(MAngle.Yaw);
        if(isShowIMUData) {
            Uart1.print("{:07.3f},{:07.3f},{:07.3f}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
        }
//        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue_0, 0 ) == pdTRUE){
//            Uart1.print("Motor output:{}\t{}\n",(notifiedValue_0>>16),(notifiedValue_0 & 0xFFFF));
//            TB6_wheel.setBVel_raw(static_cast<int16_t>(notifiedValue_0>>14));
//            TB6_wheel.setAVel_raw(static_cast<int16_t>(notifiedValue_0 & 0xFFFF));
//        }
        vel_loop_cnt++;
        if(vel_loop_cnt >= 5){
            vel_loop_cnt = 0;
            double Left_RPM = Enc_Left.getRPM();
            double Right_RPM = Enc_Right.getRPM();
            double Ave_RPM = (Left_RPM+Right_RPM)/2;
            Differ_RPM = Left_RPM - Right_RPM;
            Velocity_PID.setTunings(Velocity_kp,Velocity_ki,Velocity_kd);
            Differ_PID.setTunings(Differ_kp,Differ_ki,Differ_kd);
            Angle_target = Velocity_PID.update(Velocity_Target,Ave_RPM);
            DifferPWM = Differ_PID.update(Differ_Target,Differ_RPM);
//            Uart1.print("Angle_target: {:07.3f}\t{:07.3f}\n",Ave_RPM,Angle_target);
            if(isShowMotorRAM){
                Uart1.print("A: {:07.3f}\tB: {:07.3f}\n",Left_RPM,Right_RPM);
            }
            //roll pid
            AdaptY_PID.setTunings(Adapt_y_kp,Adapt_y_ki,0);
            float roll_error = Roll_Target - MAngle.Roll;
            static float last_target_roll = 0;
            // 检测目标角度是否跨越零点（正负号改变）
            if ((last_target_roll > 0 && Roll_Target < 0) || (last_target_roll < 0 && Roll_Target > 0)) {
                AdaptY_PID.reset(); // 清除旧的增量累加值 last_out_ 和积分项
            }
            last_target_roll = Roll_Target;
            float adjust_y = AdaptY_PID.updateIncremental(Roll_Target,MAngle.Roll);
            float geometric_comp_y;
            const float THRESHOLD_DEG = 3.0f;      // 触发补偿的 Roll 角阈值 (度)
            const float K_COMP = 0.5f;             // 补偿系数 (0.0~1.0)，建议先给 0.8，避免过冲
            // 使用平滑死区处理误差，避免补偿量突变导致舵机抽搐
            if (roll_error > 3.0f) {
                // 仅对超出阈值的部分进行正弦补偿
                geometric_comp_y = K_COMP * 55.0 * std::sin((roll_error - THRESHOLD_DEG) * 0.0174532925f);
                adjust_y += geometric_comp_y;
            }
            else if (roll_error < -3.0f) {
                geometric_comp_y = K_COMP * 55.0 * std::sin((roll_error + THRESHOLD_DEG) * 0.0174532925f);
                adjust_y += geometric_comp_y;
            }
            Left_Legheight = (Target_height - adjust_y);
            Right_Legheight = Target_height + adjust_y;
            xTaskNotifyGive(Handle_ServoControlFunc);
        }
        Angle_PID.setTunings(Angle_kp,Angle_ki,Angle_kd);
        float EvenPWM = Angle_PID.update(Angle_target,MAngle.Pitch + Angle_bias);
        int Left_PWM = static_cast<int>(std::round((EvenPWM + DifferPWM)));
        int Right_PWM = static_cast<int>(std::round((EvenPWM - DifferPWM)));
        Left_PWM = TB6612::clamp(Left_PWM,1000,-1000);
        Right_PWM = TB6612::clamp(Right_PWM,1000,-1000);
        TB6_wheel.setAVel_raw(Left_PWM);
        TB6_wheel.setBVel_raw(Right_PWM);
        taskEXIT_CRITICAL();
    }
}

/*---------------------  Servo task function begin  ---------------------*/

TaskFunction_t ServoControlFunc(){
    Servo Ser_Lift(&htim9,TIM_CHANNEL_1,
                   Servo::PhysicalToPulse(0.0f),
                   Servo::PhysicalToPulse(180.0f),
                   180.0);
    Servo Ser_Right(&htim9,TIM_CHANNEL_2,
                    Servo::PhysicalToPulse(169.0f),
                    Servo::PhysicalToPulse(11.0f),
                    180.0);
    Ser_Lift.Init();
    Ser_Right.Init();
    Ser_Lift.setLimit(0,50);
    Ser_Right.setLimit(0,50);
    uint32_t notifiedValue = 0;
    uint16_t targetAngle,moveSpeed;
    float Left_deg{},Right_deg{};
    while(1){
//        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY ) == pdTRUE){
//            targetAngle = (uint16_t)((notifiedValue >> 20) & 0xFF); // 取高 8 位
//            moveSpeed = (uint16_t)(notifiedValue & 0xFFF);     // 取低 16 位
//            Uart1.print("servoTask: {}, {}\n",targetAngle,moveSpeed);
//            if(targetAngle > 50){
//                targetAngle = 50;
//            }
//            else if(targetAngle<0){
//                targetAngle = 0;
//            }
//            if(moveSpeed == 0){
//                Ser_Lift.setAngle_Immediate(static_cast<float>(targetAngle));
//                Ser_Right.setAngle_Immediate(static_cast<float>(targetAngle));
//            } else {
//                Ser_Lift.setAngle_Smooth(static_cast<float>(targetAngle),static_cast<float>(moveSpeed));
//                Ser_Right.setAngle_Smooth(static_cast<float>(targetAngle),static_cast<float>(moveSpeed));
//            }
//        }
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)){
            float L_x,R_x;
            if(Left_Legheight>78.5)     {Left_Legheight=78.5;}
            else if(Left_Legheight<44.5){Left_Legheight=44.5;}
            if(Right_Legheight>78.5)    {Right_Legheight=78.5;}
            else if(Right_Legheight<44.5){Right_Legheight=44.5;}
            Left_deg = LegKinematics::getMotorAngleForHeight(Left_Legheight,&L_x);
            Right_deg = LegKinematics::getMotorAngleForHeight(Right_Legheight,&R_x);
            Ser_Lift.setAngle_Smooth(Left_deg-10.0f,1000);
            Ser_Right.setAngle_Smooth(Right_deg-10.0f,1000);
//        Uart1.print("servoTask: {}, {}\n",Left_deg,Right_deg);
        }
    }
}

void CPP_Main()
{

    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)LEDBlinkFunc,
                           (const char*)"LEDBlink",
                           (uint16_t)2000,
                           (void*)NULL,
                           (UBaseType_t)28,
                           (TaskHandle_t*)&Handle_LEDBlinkFunc);
    xReturn |= xTaskCreate((TaskFunction_t)ServoControlFunc,
                          (const char*)"ServoControl",
                          (uint16_t)256,
                          (void*)NULL,
                          (UBaseType_t)28,
                          (TaskHandle_t*)&Handle_ServoControlFunc);
    xReturn |= xTaskCreate((TaskFunction_t)MotionControlFunc_PID,
                           (const char*)"MotionControl",
                           (uint16_t)2500,
                           (void*)NULL,
                           (UBaseType_t)29,
                           (TaskHandle_t*)&Handle_MotionControlFunc);

    if(pdPASS == xReturn){
        Uart1.print("CPPMain: success\n");
    }
    else {
        Uart1.print("CPPMain: fail\n");
    }
}

/*---------------------  system interrupt callback function begin  ---------------------*/

//spi interrupt callback
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    if (hspi == nRF.getSPIHandle()) { // 替换为你的 SPI 实例
        nRF.isrSpiDmaCompleteHandler();
    }
}
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == nRF.getSPIHandle()) {
        nRF.isrSpiDmaCompleteHandler();
    }
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == nRF.getSPIHandle()) {
        nRF.isrSpiDmaCompleteHandler();
    }
}

// EXTI interrupt callback
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == nRF.getIRQGPIOPort()) { // 替换为你在 CubeMX 中定义的引脚宏
        nRF.isrExtiHandler();
    }
}
