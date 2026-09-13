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
#include "CtrlAlgorithm/BalanceCompensation.hpp"
#include "CtrlAlgorithm/BalanceStartupGate.hpp"
#include "MotionParameterCommands.hpp"
#include "MotionParameterStorage.hpp"
#include "MotionStorageInterlock.hpp"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"



/* My variables define BEGIN */
LkUart<> Uart1(&huart1);
volatile bool isShowIMUData;
volatile bool isShowMotorRAM;
/// PID
// angle-pid
// Settings are shared only under taskENTER_CRITICAL; aliases retain debugger names.
MotionSettings::Parameters Motion_parameters{};
MotionSettings::Parameters Saved_parameters{};
bool Have_saved_parameters{};
MotionStorageInterlock Storage_interlock;
float& Angle_kp_mid = Motion_parameters.angle.kp;
volatile float Angle_kp{MotionSettings::effectiveAngleKp(
    MotionSettings::Parameters{}.angle.kp, BalanceCompensation::minimum_leg_height_mm)};
float& Angle_ki = Motion_parameters.angle.ki;
float& Angle_kd = Motion_parameters.angle.kd;
// anglebias commands calibrate the minimum-height baseline; only MotionControl
// writes the effective bias used by the pitch PID (向前则减小).
float& Angle_bias_min = Motion_parameters.minimum_pitch_bias;
volatile float Angle_bias{BalanceCompensation::default_minimum_bias_degrees};
volatile bool Control_armed{};
volatile bool Control_imu_valid{};
volatile float Control_pitch_error{};
volatile int Control_left_pwm{};
volatile int Control_right_pwm{};
// velocity-pid
float& Velocity_kp = Motion_parameters.velocity.kp;
float& Velocity_ki = Motion_parameters.velocity.ki;
float& Velocity_kd = Motion_parameters.velocity.kd;
volatile float Velocity_Target{};
//differ-pid
float& Differ_kp = Motion_parameters.difference.kp;
float& Differ_ki = Motion_parameters.difference.ki;
float& Differ_kd = Motion_parameters.difference.kd;
volatile float Differ_Target{};
//adapt_y pid
float& Adapt_y_kp = Motion_parameters.roll.kp;
float& Adapt_y_ki = Motion_parameters.roll.ki;
float& Adapt_y_kd = Motion_parameters.roll.kd;
/// real Angle value
volatile float EAngle_print[3]{};
/// Leg height
volatile float Left_Legheight{BalanceCompensation::minimum_leg_height_mm};
volatile float Right_Legheight{BalanceCompensation::minimum_leg_height_mm};
float& Target_height = Motion_parameters.leg_height;
float& Roll_Target = Motion_parameters.roll_target;
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

static std::string_view commandText(etl::string_view text)
{
    return MotionSettings::trim({text.data(), text.size()});
}

static bool tuneMotion(std::string_view name, etl::string_view args)
{
    taskENTER_CRITICAL();
    auto parameters = Motion_parameters;
    taskEXIT_CRITICAL();
    const bool ok = MotionSettings::applyTuning(parameters, name, commandText(args));
    if (ok) {
        taskENTER_CRITICAL();
        Motion_parameters = parameters;
        taskEXIT_CRITICAL();
    }
    const etl::string_view label(name.data(), name.size());
    if (ok) Uart1.print("{}: ok (RAM; use save to persist)\n", label);
    else Uart1.print("{}: invalid arguments\n", label);
    return ok;
}

static void saveMotion(etl::string_view args)
{
    const auto mode = MotionSettings::parseSaveMode(commandText(args));
    if (mode == MotionSettings::SaveMode::invalid) {
        Uart1.print("save: usage: save [all|recycle]\n");
        return;
    }
    taskENTER_CRITICAL();
    const bool allowed = Storage_interlock.begin(Control_armed, Control_left_pwm, Control_right_pwm);
    const auto parameters = Motion_parameters;
    taskEXIT_CRITICAL();
    if (!allowed) {
        Uart1.print("save: busy; support robot, use control off, then retry\n");
        return;
    }

    // Interrupts/HAL timeouts stay enabled. The control task holds PWM at zero
    // and resets its 500 ms startup gate throughout this operation.
    const auto result = MotionSettings::saveToFlash(parameters, mode == MotionSettings::SaveMode::recycle);
    // Also refresh after errors: an interrupted explicit recycle can erase history.
    Have_saved_parameters = MotionSettings::loadFromFlash(Saved_parameters);
    taskENTER_CRITICAL();
    Storage_interlock.finish();
    taskEXIT_CRITICAL();
    switch (result) {
    case MotionSettings::SaveResult::saved: Uart1.print("save: ok (all motion parameters)\n"); break;
    case MotionSettings::SaveResult::unchanged: Uart1.print("save: unchanged (no flash write)\n"); break;
    case MotionSettings::SaveResult::full: Uart1.print("save: full; use save recycle to erase journal and save\n"); break;
    case MotionSettings::SaveResult::invalid: Uart1.print("save: invalid parameters\n"); break;
    case MotionSettings::SaveResult::io_error: Uart1.print("save: flash error; RAM settings retained\n"); break;
    }
}

static void showMotion(etl::string_view args)
{
    if (!commandText(args).empty()) {
        Uart1.print("params: usage: params\n");
        return;
    }
    taskENTER_CRITICAL();
    const auto p = Motion_parameters;
    const float kp = Angle_kp, bias = Angle_bias;
    const float left = Left_Legheight, right = Right_Legheight;
    const bool armed = Control_armed;
    const bool enabled = Storage_interlock.enabled();
    taskEXIT_CRITICAL();
    const bool dirty = !Have_saved_parameters || MotionSettings::encode(p) != MotionSettings::encode(Saved_parameters);
    Uart1.print("params: flash_valid={} unsaved={} armed={} enabled={}\n", Have_saved_parameters, dirty, armed, enabled);
    Uart1.print("anglebias min={:.4f} effective={:.4f}\n", p.minimum_pitch_bias, bias);
    Uart1.print("anglepid p_mid={:.4f} i={:.6f} d={:.4f} p_effective={:.4f}\n", p.angle.kp, p.angle.ki, p.angle.kd, kp);
    Uart1.print("velocitypid p={:.6f} i={:.6f} d={:.6f}\n", p.velocity.kp, p.velocity.ki, p.velocity.kd);
    Uart1.print("differpid p={:.6f} i={:.6f} d={:.6f}\n", p.difference.kp, p.difference.ki, p.difference.kd);
    Uart1.print("rollpid/legpid p={:.6f} i={:.6f} d={:.6f}\n", p.roll.kp, p.roll.ki, p.roll.kd);
    Uart1.print("legheight={:.4f} target_roll={:.4f} left={:.4f} right={:.4f}\n", p.leg_height, p.roll_target, left, right);
}


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
            {"anglepid",[](etl::string_view args){ tuneMotion("anglepid", args); }},
            {"velocitypid",[](etl::string_view args){ tuneMotion("velocitypid", args); }},
            {"differpid",[](etl::string_view args){ tuneMotion("differpid", args); }},
            {"rollpid",[](etl::string_view args){ tuneMotion("rollpid", args); }},
            {"legpid",[](etl::string_view args){ tuneMotion("legpid", args); }},
            {"save",[](etl::string_view args){ saveMotion(args); }},
            {"params",[](etl::string_view args){ showMotion(args); }},
            {"control",[](etl::string_view args){
                const auto mode = commandText(args);
                if (mode != "off" && mode != "on") {
                    Uart1.print("control: usage: control off|on\n");
                    return;
                }
                taskENTER_CRITICAL();
                Storage_interlock.setEnabled(mode == "on");
                taskEXIT_CRITICAL();
                if (mode == "off") Uart1.print("control: off requested; wait for params armed=false before save\n");
                else Uart1.print("control: on; waiting for normal startup conditions\n");
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
                if (tuneMotion("legheight", args)) {
                    float result_x{};
                    const float height = Target_height;
                    const float result_deg = LegKinematics::getMotorAngleForHeight(height, &result_x);
                    const float bias_p = ((-0.000155f * height + 0.03882f) * height - 3.001f) * height + 83.25f;
                    Uart1.print("Servo angel: {:07.3f} {:07.3f} {:07.3f}\n", result_deg, result_x, bias_p);
                }
            }},
            {"target_roll",[](etl::string_view args){ tuneMotion("target_roll", args); }},
            {"VandD",[](etl::string_view args){
                auto text = commandText(args);
                float velocity{}, difference{};
                if (MotionSettings::parseFloat(text, difference) && MotionSettings::parseFloat(text, velocity) && text.empty()) {
                    taskENTER_CRITICAL();
                    Differ_Target = difference;
                    Velocity_Target = velocity;
                    taskEXIT_CRITICAL();
                } else Uart1.print("VandD: invalid arguments\n");
            }},
            {"R",[](etl::string_view args){
                auto text = commandText(args);
                float velocity{}, difference{}, height{}, roll{};
                if (MotionSettings::parseFloat(text, difference) && MotionSettings::parseFloat(text, velocity) &&
                    MotionSettings::parseFloat(text, roll) && MotionSettings::parseFloat(text, height) && text.empty()) {
                    taskENTER_CRITICAL();
                    Differ_Target = difference;
                    Velocity_Target = velocity;
                    Roll_Target = roll;
                    Target_height = BalanceCompensation::clampLegHeight(height);
                    taskEXIT_CRITICAL();
                } else Uart1.print("R: invalid arguments\n");
            }},
            {"anglebias",[](etl::string_view args){ tuneMotion("anglebias", args); }},
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
        if(CMD_que.full()) return;
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
            auto text = commandText(cmd);
            const auto name = MotionSettings::takeToken(text);
            Uart_CMD.command = etl::string_view(name.data(), name.size());
            Uart_CMD.args = etl::string_view(text.data(), text.size());
            if(!name.empty()){
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
    PID Differ_PID(0,0,0,-500,500,-100,100);
    PID AdaptY_PID(0,0,0,-78,78,-100,100);
    BalanceStartupGate startup_gate;
    float Differ_RPM, Angle_target{},DifferPWM{};
    float last_target_roll = 0;
    //sensor
    MPU6050 IMU_Main(&hi2c1,{MPU6050::GyroRange_t::G1000,MPU6050::AccRange_t::A4,static_cast<uint16_t>(MPU6050::ms_toHZ(xFrequency)),{0,0,0}});
    MPU6050::EulerAngle MAngle{};
    double IMU_Gyro[3]{};
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
    const bool imu_initialized = IMU_Main.Init();
    if(imu_initialized){
        Uart1.print("MPU: success\n");
    }
    else{
        Uart1.print("MPU: fail\n");
    }
    Enc_Left.clearCounter();
    Enc_Right.clearCounter();
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Keep the scheduler and HAL timeout ticks running during sensor I/O.
        const bool imu_valid = imu_initialized && IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro) &&
            std::isfinite(MAngle.Roll) && std::isfinite(MAngle.Pitch) && std::isfinite(MAngle.Yaw) &&
            std::isfinite(IMU_Gyro[0]) && std::isfinite(IMU_Gyro[1]) && std::isfinite(IMU_Gyro[2]);
        taskENTER_CRITICAL();
        Control_imu_valid = imu_valid;
        EAngle_print[0] = static_cast<volatile float>(MAngle.Roll); EAngle_print[1] = static_cast<volatile float>(MAngle.Pitch);EAngle_print[2] = static_cast<volatile float>(MAngle.Yaw);
        if(isShowIMUData) {
            Uart1.print("{:07.3f},{:07.3f},{:07.3f}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
        }
//        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue_0, 0 ) == pdTRUE){
//            Uart1.print("Motor output:{}\t{}\n",(notifiedValue_0>>16),(notifiedValue_0 & 0xFFFF));
//            TB6_wheel.setBVel_raw(static_cast<int16_t>(notifiedValue_0>>14));
//            TB6_wheel.setAVel_raw(static_cast<int16_t>(notifiedValue_0 & 0xFFFF));
//        }
        const float common_height = BalanceCompensation::clampLegHeight(Target_height);
        const float gate_height = Control_armed
            ? BalanceCompensation::averageLegHeight(Left_Legheight, Right_Legheight) : common_height;
        const float corrected_pitch = static_cast<float>(MAngle.Pitch) +
            BalanceCompensation::pitchBias(Angle_bias_min, gate_height);
        const float gyro_rate = static_cast<float>(std::sqrt(
            IMU_Gyro[0]*IMU_Gyro[0] + IMU_Gyro[1]*IMU_Gyro[1] + IMU_Gyro[2]*IMU_Gyro[2]) * 57.295779513);
        if (Storage_interlock.consumeReset()) {
            startup_gate.reset();
            xLastWakeTime = xTaskGetTickCount();
        }
        Control_armed = Storage_interlock.canRun() && startup_gate.update(imu_valid, corrected_pitch, static_cast<float>(MAngle.Roll),
                                           gyro_rate, Velocity_Target, Differ_Target);
        if (!Control_armed) {
            Angle_PID.reset();
            Velocity_PID.reset();
            Differ_PID.reset();
            AdaptY_PID.reset();
            Angle_target = DifferPWM = 0.0f;
            last_target_roll = Roll_Target;
            Left_Legheight = common_height;
            Right_Legheight = common_height;
            Angle_bias = BalanceCompensation::pitchBias(Angle_bias_min, common_height);
            Angle_kp = MotionSettings::effectiveAngleKp(Angle_kp_mid, common_height);
            Control_pitch_error = static_cast<float>(MAngle.Pitch) + Angle_bias;
            Control_left_pwm = 0;
            Control_right_pwm = 0;
            TB6_wheel.setAVel_raw(0);
            TB6_wheel.setBVel_raw(0);
            if (++vel_loop_cnt >= 5) {
                vel_loop_cnt = 0;
                // Consume encoder deltas while idle so re-entry has no stale velocity sample.
                Enc_Left.getRPM();
                Enc_Right.getRPM();
                xTaskNotifyGive(Handle_ServoControlFunc);
            }
            taskEXIT_CRITICAL();
            continue;
        }
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
            AdaptY_PID.setTunings(Adapt_y_kp,Adapt_y_ki,Adapt_y_kd);
            float roll_error = Roll_Target - MAngle.Roll;
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
            // Publish bounded targets before either balance or servo calculations.
            Left_Legheight = BalanceCompensation::clampLegHeight(Target_height - adjust_y);
            Right_Legheight = BalanceCompensation::clampLegHeight(Target_height + adjust_y);
            xTaskNotifyGive(Handle_ServoControlFunc);
        }
        const float Y_avg = BalanceCompensation::averageLegHeight(Left_Legheight, Right_Legheight);
        Angle_bias = BalanceCompensation::pitchBias(Angle_bias_min, Y_avg);
        Angle_kp = MotionSettings::effectiveAngleKp(Angle_kp_mid, Y_avg);
        Angle_PID.setTunings(Angle_kp,Angle_ki,Angle_kd);
        float EvenPWM = Angle_PID.update(Angle_target,MAngle.Pitch + Angle_bias);
        Control_pitch_error = static_cast<float>(MAngle.Pitch) + Angle_bias;
        int Left_PWM = static_cast<int>(std::round((EvenPWM + DifferPWM)));
        int Right_PWM = static_cast<int>(std::round((EvenPWM - DifferPWM)));
        Left_PWM = TB6612::clamp(Left_PWM,1000,-1000);
        Right_PWM = TB6612::clamp(Right_PWM,1000,-1000);
        Control_left_pwm = Left_PWM;
        Control_right_pwm = Right_PWM;
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
            // Take a consistent pair; MotionControl owns target updates and limits.
            taskENTER_CRITICAL();
            const float left_height = Left_Legheight;
            const float right_height = Right_Legheight;
            taskEXIT_CRITICAL();
            Left_deg = LegKinematics::getMotorAngleForHeight(left_height,&L_x);
            Right_deg = LegKinematics::getMotorAngleForHeight(right_height,&R_x);
            Ser_Lift.setAngle_Smooth(Left_deg-10.0f,1000);
            Ser_Right.setAngle_Smooth(Right_deg-10.0f,1000);
//        Uart1.print("servoTask: {}, {}\n",Left_deg,Right_deg);
        }
    }
}

void CPP_Main()
{
    // Load before creating any task: no controller can see a partially restored set.
    Have_saved_parameters = MotionSettings::loadFromFlash(Motion_parameters);
    Saved_parameters = Motion_parameters;
    Left_Legheight = Target_height;
    Right_Legheight = Target_height;
    Angle_bias = BalanceCompensation::pitchBias(Angle_bias_min, Target_height);
    Angle_kp = MotionSettings::effectiveAngleKp(Angle_kp_mid, Target_height);
    if (Have_saved_parameters) Uart1.print("params: loaded from flash\n");
    else Uart1.print("params: compiled defaults (no valid flash record)\n");

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
