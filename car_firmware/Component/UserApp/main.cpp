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
/// cpp etl include
//#include "etl/memory.h"
//#include "etl/pool.h"
#include "etl/unordered_map.h"
#include "etl/format.h"
/// user library include
#include "cpp_Interface.h"
#include "main.h"
#include "AppTask.hpp"
#include "BoardHardware.hpp"
#include "RuntimeStatus.hpp"
///cpp User library include
#include "TaskReactor.hpp"
#include "CtrlAlgorithm/LQR.hpp"
#include "CtrlAlgorithm/PID.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"



/* My variables define BEGIN */
volatile bool isShowIMUData;
volatile bool isShowMotorRAM;
/// PID
// angle-pid
volatile float Angle_kp{70.0f};
volatile float Angle_ki{};
volatile float Angle_kd{60.0f};
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
volatile bool isNRF_print{};
const volatile float* NRF_print[4]{reinterpret_cast<const volatile float *>(&EAngle_print[0]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[1]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[2]),
                                   reinterpret_cast<const volatile float *>(&Angle_kp)};
/* My variables define END */

namespace {

bsp::BoardHardware* g_hardware = nullptr;
app::RuntimeStatus g_runtime_status;
app::InitializationReport g_initialization_report;
app::AppTask* g_servo_task = nullptr;
app::AppTask* g_motion_task = nullptr;

bsp::BoardHardware& hardware()
{
    configASSERT(g_hardware != nullptr);
    return *g_hardware;
}

LkUart<>& command_uart()
{
    return hardware().command_uart();
}

NRF24L01P& radio()
{
    return hardware().radio();
}

bool hardware_ready(bsp::HardwareModuleId id)
{
    return g_initialization_report.succeeded(bsp::module_id(id));
}

const char* system_state_name(app::SystemState state)
{
    switch (state) {
    case app::SystemState::booting: return "booting";
    case app::SystemState::ready: return "ready";
    case app::SystemState::initialization_failed: return "init-failed";
    case app::SystemState::task_failed: return "task-failed";
    case app::SystemState::runtime_fault: return "runtime-fault";
    }
    return "unknown";
}

} // namespace


/*---------------------  define task function begin  ---------------------*/
using CommandHandler = std::function<void(etl::string_view)>;

void HeartbeatFunc(void*)
{
    app::SystemState previous_state = app::SystemState::booting;
    uint8_t phase = 0U;

    while (true) {
        const app::SystemState state = g_runtime_status.state();
        if (state != previous_state) {
            previous_state = state;
            phase = 0U;
        }

        bool led_on = false;
        uint32_t duration_ms = 100U;
        switch (state) {
        case app::SystemState::booting:
            led_on = (phase == 0U);
            duration_ms = 100U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        case app::SystemState::ready:
            led_on = (phase == 0U);
            duration_ms = led_on ? 80U : 920U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        case app::SystemState::initialization_failed:
            led_on = (phase == 0U || phase == 2U);
            duration_ms = (phase == 3U) ? 640U : 120U;
            phase = static_cast<uint8_t>((phase + 1U) % 4U);
            break;
        case app::SystemState::task_failed:
            led_on = (phase == 0U || phase == 2U || phase == 4U);
            duration_ms = (phase == 5U) ? 520U : 120U;
            phase = static_cast<uint8_t>((phase + 1U) % 6U);
            break;
        case app::SystemState::runtime_fault:
            led_on = (phase == 0U);
            duration_ms = 80U;
            phase = static_cast<uint8_t>((phase + 1U) % 2U);
            break;
        }

        // PC13 LED is active-low on the WL1 controller board.
        HAL_GPIO_WritePin(GPIOC,
                          GPIO_PIN_13,
                          led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

void CommandServiceFunc(void*)
{
    TaskReactor t1;
    TaskReactor::strCMD_t Uart_CMD;
    uint8_t NRF_Tx_Num[NRF24L01P::PACKET_WIDTH]{};
    uint8_t NRF_Rx_Num[NRF24L01P::PACKET_WIDTH]{};
    etl::string_view NRF_RxStr{};
    etl::unordered_map<etl::string_view,CommandHandler,25,57> cmdMap = {
            {"ping",[](etl::string_view){
                command_uart().print("pong state={} control={}\n",
                                     system_state_name(g_runtime_status.state()),
                                     g_runtime_status.control_enabled() ? "on" : "off");
            }},
            {"status",[](etl::string_view){
                command_uart().print("status={} control={} hw_fail={} task_fail={}\n",
                                     system_state_name(g_runtime_status.state()),
                                     g_runtime_status.control_enabled() ? "on" : "off",
                                     g_runtime_status.hardware_failed_mask(),
                                     g_runtime_status.task_failed_mask());
            }},
            {"motor",[](etl::string_view args){
                uint16_t vL = 0, vR = 0;
                if(TaskReactor::parseStrArg(args,vL) && TaskReactor::parseStrArg(args,vR)){
                    if (!g_runtime_status.control_enabled() || g_motion_task == nullptr ||
                        !g_motion_task->is_running()) {
                        command_uart().print("motor rejected: control is in safe mode\n");
                        return;
                    }
                    command_uart().print("motor: {}\t{}\n",vL,vR);
                    uint32_t notifyValue = (vL << 16) | (vR & 0xFFFF);
                    (void)g_motion_task->notify_value(notifyValue);
                }
                else{
                    command_uart().print("Command \"motor\": Useless parameters\n");
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
                if (!hardware_ready(bsp::HardwareModuleId::radio)) {
                    command_uart().print("nrfsend rejected: radio unavailable\n");
                    return;
                }
                NRF24L01P::str_touint8(args, NRF_Tx_Num);
                radio().send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH);
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
                    command_uart().print("Servo angel: {:07.3f} {:07.3f} {:07.3f}\n",result_deg,result_x,bais_p);
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
            {"R",[](etl::string_view args){
                float target_v{},target_d{},target_h{},target_r{};
                if(TaskReactor::parseStrArg(args,target_d) && TaskReactor::parseStrArg(args,target_v) &&
                    TaskReactor::parseStrArg(args,target_r) && TaskReactor::parseStrArg(args,target_h)){
                    Differ_Target = target_d;
                    Velocity_Target = target_v;
                    Roll_Target = target_r;
                    Target_height = target_h;
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
    if (hardware_ready(bsp::HardwareModuleId::command_uart)) {
        t1.connect(&command_uart(),
                   &LkUart<>::signal_RxComplete,
                   [&CMD_que](etl::string<128>& rxmes) {
                       if (CMD_que.full()) {
                           command_uart().print("command dropped: queue full\n");
                           return;
                       }
                       if (rxmes.size() > 32U) {
                           CMD_que.push(rxmes.substr(0, 32));
                       } else {
                           CMD_que.push(rxmes);
                       }
                   });
    }

    if (hardware_ready(bsp::HardwareModuleId::radio)) {
        t1.connect(&radio(),
                   &NRF24L01P::signal_IRQEvent,
                   [&NRF_Rx_Num, &NRF_RxStr, &CMD_que](NRF24L01P::Status_t& curStatus) {
                       if (curStatus.RX_DR) {
                           radio().tryReceive(NRF_Rx_Num);
                           NRF24L01P::uint8_tostr(NRF_RxStr, NRF_Rx_Num);
                           if (!CMD_que.full()) {
                               CMD_que.push(etl::string<32>(NRF_RxStr));
                           }
                       }
                       if (curStatus.TX_DS) {
                           command_uart().print("nRF: send success\n");
                       }
                       if (curStatus.MAX_RT) {
                           command_uart().print("nRF: send fail\n");
                       }
                   });
    }

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
                    command_uart().print("receive: {}\n",cmd);
                }
            }
            else{
                command_uart().print("receive: {}\n",cmd);
            }
        }
    },
    [&NRF_Tx_Num](){
        if(isNRF_print && hardware_ready(bsp::HardwareModuleId::radio)){
            NRF24L01P::args_touint8s(NRF_Tx_Num,NRF_print);
            radio().send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH);
        }
    });
}

/*---------------------  LQR task function begin  ---------------------*/
void MotionControlFunc(void*){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;   //(ms)
    uint32_t notifiedValue_0{};
    ///  c++ variable
    auto& IMU_Main = hardware().imu();
    MPU6050::EulerAngle MAngle;
    //    double IMU_Acc[3];
    double IMU_Gyro[3];
    LQR Ctrl_cal(std::forward<double[4]>({-4.569790f, -4.472503f, -0.000000f, 0.000000f}));
    auto& Enc_Left = hardware().left_encoder();
    auto& Enc_Right = hardware().right_encoder();
    auto& TB6_wheel = hardware().wheel_motor();
    TB6_wheel.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::A), TB6612::Direction::Negative);
    TB6_wheel.setA_DeadZone(0);TB6_wheel.setB_DeadZone(0);
    double LQRPos_Left{};
    double LQRPos_Right{};
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (!g_runtime_status.control_enabled()) {
            TB6_wheel.forceStop();
            continue;
        }
        taskENTER_CRITICAL();
        static double filtered_RPM_Left = 0;
        static double filtered_RPM_Right = 0;
        if (!IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro)) {
            taskEXIT_CRITICAL();
            g_runtime_status.enter_runtime_fault();
            hardware().force_safe_outputs();
            command_uart().print("[runtime][FAIL] imu read; control stopped\n");
            continue;
        }
        if(isShowIMUData) {
            command_uart().print("{:07.3f},{:07.3f},{:07.3f}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
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
            command_uart().print("A: {:07.3f}\tB: {:07.3f}\n",RPM_Left_Raw,RPM_Right_Raw);
        }
        if(xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue_0, 0 ) == pdTRUE){
            command_uart().print("Motor output:{}\t{}\n",(notifiedValue_0>>14),(notifiedValue_0 & 0xFFFF));
            TB6_wheel.setBVel_raw(static_cast<int16_t>(notifiedValue_0>>14));
            TB6_wheel.setAVel_raw(static_cast<int16_t>(notifiedValue_0 & 0xFFFF));
        }
//        TB6_wheel.setAVel_raw(-NeededPWM_Right);
//        TB6_wheel.setBVel_raw(-NeededPWM_Left);
        taskEXIT_CRITICAL();
    }
}


/*---------------------  PID task function begin  ---------------------*/
void MotionControlFunc_PID(void*){
    ///  FreeRtos variable
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;   //(ms)
    uint8_t vel_loop_cnt = 0;
    uint8_t imu_read_failures = 0U;
    ///  c++ variable
    //  PID
    PID Angle_PID(70.0f,0,51.0f,-1000,1000,-100,100);
    PID Velocity_PID(0.04,0.006,0,-10,10,-100,100);
    PID Differ_PID(0,0,0,-500,500,-100,100);
    PID AdaptY_PID(0,0,0,-78,78,-100,100);
    float Differ_RPM, Angle_target{},DifferPWM{};
    //sensor
    auto& IMU_Main = hardware().imu();
    MPU6050::EulerAngle MAngle;
    double IMU_Gyro[3];
    auto& Enc_Left = hardware().left_encoder();
    auto& Enc_Right = hardware().right_encoder();
    auto& TB6_wheel = hardware().wheel_motor();
    xLastWakeTime = xTaskGetTickCount();        //get now system tick to delay a period
    while(1){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (!g_runtime_status.control_enabled()) {
            TB6_wheel.forceStop();
            continue;
        }
        taskENTER_CRITICAL();
        // read now Leg height to calculate pitch angle compensate and angle_kp
        float Y_avg = (Left_Legheight + Left_Legheight) / 2.0f;
        Angle_bias = (0.01026f * Y_avg * Y_avg) - (1.258f * Y_avg) + 48.24f;
        Angle_kp = (0.3f*Y_avg) + 56.9;
        // get IMU euler angle
        if (!IMU_Main.getEulerAngleGyro(MAngle,IMU_Gyro)) {
            taskEXIT_CRITICAL();
            TB6_wheel.forceStop();
            if (++imu_read_failures >= 3U) {
                g_runtime_status.enter_runtime_fault();
                hardware().force_safe_outputs();
                command_uart().print("[runtime][FAIL] imu read; control stopped\n");
            }
            continue;
        }
        imu_read_failures = 0U;
        EAngle_print[0] = static_cast<float>(MAngle.Roll);
        EAngle_print[1] = static_cast<float>(MAngle.Pitch);
        EAngle_print[2] = static_cast<float>(MAngle.Yaw);
        if(isShowIMUData) {
            command_uart().print("{:07.3f},{:07.3f},{:07.3f}\n", MAngle.Roll, MAngle.Pitch, MAngle.Yaw);
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
                command_uart().print("A: {:07.3f}\tB: {:07.3f}\n",Left_RPM,Right_RPM);
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
            if (g_servo_task != nullptr) {
                (void)g_servo_task->notify_give();
            }
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

void ServoControlFunc(void*){
    auto& Ser_Lift = hardware().left_servo();
    auto& Ser_Right = hardware().right_servo();
    float Left_deg{},Right_deg{};
    while(1){
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)){
            if (!g_runtime_status.control_enabled()) {
                Ser_Lift.stop();
                Ser_Right.stop();
                continue;
            }
            float L_x,R_x;
            if(Left_Legheight>78.5)     {Left_Legheight=78.5;}
            else if(Left_Legheight<44.5){Left_Legheight=44.5;}
            if(Right_Legheight>78.5)    {Right_Legheight=78.5;}
            else if(Right_Legheight<44.5){Right_Legheight=44.5;}
            Left_deg = LegKinematics::getMotorAngleForHeight(Left_Legheight,&L_x);
            Right_deg = LegKinematics::getMotorAngleForHeight(Right_Legheight,&R_x);
            Ser_Lift.setAngle_Smooth(Left_deg-10.0f,1000);
            Ser_Right.setAngle_Smooth(Right_deg-10.0f,1000);
        }
    }
}

namespace {

void initialization_observer(const app::InitializationStep& step,
                             bool succeeded,
                             void* context)
{
    auto* board = static_cast<bsp::BoardHardware*>(context);
    if (board == nullptr) {
        return;
    }

    if (succeeded) {
        board->command_uart().print("[init][ OK ] {}\n", step.name);
    } else {
        board->command_uart().print("[init][FAIL] {}\n", step.name);
    }
    // Startup diagnostics must not overrun the fixed-depth asynchronous TX
    // queue when several modules finish immediately one after another.
    vTaskDelay(pdMS_TO_TICKS(3));
}

bool start_task(app::AppTask& task, void* context)
{
    const bool started = task.start(context);
    g_runtime_status.record_task_result(task.id(), started);
    if (started) {
        command_uart().print("[task][ OK ] {}\n", task.name());
    } else {
        command_uart().print("[task][FAIL] {}\n", task.name());
    }
    vTaskDelay(pdMS_TO_TICKS(3));
    return started;
}

} // namespace

void CPP_Main()
{
    static bsp::BoardHardware board;
    static app::AppTask heartbeat_task({app::TaskId::heartbeat,
                                        "Heartbeat",
                                        192,
                                        tskIDLE_PRIORITY + 1,
                                        HeartbeatFunc});
    static app::AppTask command_task({app::TaskId::command_service,
                                      "CommandService",
                                      2000,
                                      28,
                                      CommandServiceFunc});
    static app::AppTask servo_task({app::TaskId::servo_control,
                                    "ServoControl",
                                    256,
                                    28,
                                    ServoControlFunc});
    static app::AppTask motion_task({app::TaskId::motion_control,
                                     "MotionControl",
                                     2500,
                                     29,
                                     MotionControlFunc_PID});

    g_hardware = &board;
    g_servo_task = &servo_task;
    g_motion_task = &motion_task;
    g_runtime_status.reset();
    board.force_safe_outputs();

    command_uart().print("[app] WL1 startup begin\n");
    const bool heartbeat_started = start_task(heartbeat_task, nullptr);

    const auto initialization_plan =
        bsp::HardwareFactory::create_initialization_plan(board);
    app::InitializationManager initialization_manager;
    g_initialization_report = initialization_manager.initialize_all(
        initialization_plan.data(),
        initialization_plan.size(),
        initialization_observer,
        &board);
    g_runtime_status.publish_initialization_report(g_initialization_report);

    const bool command_channel_available =
        hardware_ready(bsp::HardwareModuleId::command_uart) ||
        hardware_ready(bsp::HardwareModuleId::radio);
    bool command_started = true;
    if (command_channel_available) {
        command_started = start_task(command_task, nullptr);
    } else {
        command_uart().print("[task][SKIP] CommandService: no command channel\n");
    }

    bool servo_started = false;
    bool motion_started = false;
    const bool may_start_control = g_initialization_report.all_succeeded() &&
                                   heartbeat_started && command_started &&
                                   g_runtime_status.task_failed_mask() == 0U;
    if (may_start_control) {
        servo_started = start_task(servo_task, nullptr);
        if (servo_started) {
            g_runtime_status.enable_control(true);
            motion_started = start_task(motion_task, nullptr);
        }
    }

    if (!may_start_control || !servo_started || !motion_started) {
        g_runtime_status.enable_control(false);
        board.force_safe_outputs();
    }

    if (g_runtime_status.task_failed_mask() != 0U) {
        g_runtime_status.set_state(app::SystemState::task_failed);
    } else if (!g_initialization_report.all_succeeded()) {
        g_runtime_status.set_state(app::SystemState::initialization_failed);
    } else if (g_runtime_status.control_enabled()) {
        g_runtime_status.set_state(app::SystemState::ready);
    } else {
        g_runtime_status.set_state(app::SystemState::task_failed);
    }

    command_uart().print("[app] state={} control={} hw_fail={} task_fail={}\n",
                         system_state_name(g_runtime_status.state()),
                         g_runtime_status.control_enabled() ? "on" : "off",
                         g_runtime_status.hardware_failed_mask(),
                         g_runtime_status.task_failed_mask());
}
