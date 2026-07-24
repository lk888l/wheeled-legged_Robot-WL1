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
#include "usart.h"
#include "spi.h"
#include "RemoteDisplay.hpp"
#include "RemoteControlState.hpp"
///cpp User library include
#include "TB6612.h"
#include "NRF24L01P.hpp"
#include "Joystick.hpp"
#include "LkUart.hpp"
#include "Button.hpp"
#include "TaskReactor.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"
//freeRTOS library include
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"


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
NRF24L01P nRF(&hspi2,GPIOA,GPIO_PIN_8,GPIOB,GPIO_PIN_12,GPIOA,GPIO_PIN_12);
RemoteControlState RemoteCommands;
volatile bool isNRF_print{};
const volatile float* NRF_print[4]{reinterpret_cast<const volatile float *>(&EAngle_print[0]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[1]),
                                   reinterpret_cast<const volatile float *>(&EAngle_print[2]),
                                   reinterpret_cast<const volatile float *>(&Angle_kp)};
Joystick V_Joy(0,4095,2048,300,-100,100);
Joystick D_Joy(0,4095,2048,300,-100,100);
Joystick R_Joy(0,4095,2048,300,-18,18);
Joystick H_Joy(0,4095,2048,300,44.5,78.5);
/* My variables define END */

// freeRTOS variably define
TaskHandle_t Handle_LEDBlinkFunc = nullptr;
TaskHandle_t Handle_ServoControlFunc = nullptr;
TaskHandle_t Handle_MotionControlFunc = nullptr;
TaskHandle_t Handle_GPIOReadFunc = nullptr;
TaskHandle_t Handle_RemoteDisplayFunc = nullptr;


/*---------------------  define task function begin  ---------------------*/
using CommandHandler = std::function<void(etl::string_view)>;
TaskFunction_t LEDBlinkFunc(){
    TaskReactor t1;
    TaskReactor::strCMD_t Uart_CMD;
    uint8_t NRF_Tx_Num[NRF24L01P::PACKET_WIDTH]{};
    uint8_t NRF_Rx_Num[NRF24L01P::PACKET_WIDTH]{};
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
            {"VandD",[](etl::string_view args){
                float target_v{},target_d{};
                if(TaskReactor::parseStrArg(args,target_d) && TaskReactor::parseStrArg(args,target_v)){
                    Differ_Target = target_d;
                    Velocity_Target = target_v;
                }
            }},
            {"HandR",[](etl::string_view args){
                float target_r{},target_h{};
                if(TaskReactor::parseStrArg(args,target_r) && TaskReactor::parseStrArg(args,target_h)){
                    Roll_Target = target_r;
                    Target_height = target_h;
                }
            }},
            {"HandR",[](etl::string_view args){
                float target_r{},target_h{};
                if(TaskReactor::parseStrArg(args,target_r) && TaskReactor::parseStrArg(args,target_h)){
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
    /// Init NRF
    bool radioReady = nRF.Init() && nRF.start_RxMode();
    if (!radioReady) {
        Uart1.print("nRF: init failed, retrying\n");
    }
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
    t1.taskLoop(pdMS_TO_TICKS(50),[&CMD_que,&Uart_CMD,&cmdMap](){
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
[&NRF_Tx_Num,&radioReady](){
        static std::uint8_t recoveryTicks = 0U;
        if (!radioReady) {
            if (++recoveryTicks < 20U) {
                return;
            }
            recoveryTicks = 0U;
            radioReady = nRF.Init() && nRF.start_RxMode();
            if (!radioReady) {
                return;
            }
            Uart1.print("nRF: recovered\n");
        }

        if(isNRF_print){
            NRF24L01P::args_touint8s(NRF_Tx_Num,NRF_print);
        } else {
            char buffer[NRF24L01P::PACKET_WIDTH]{};
            const auto command = RemoteCommands.snapshot();
            const int length = snprintf(buffer, sizeof(buffer), "R %.1f %.1f %.1f %.1f",
                                        command.turn, -command.speed,
                                        command.roll_degrees, command.leg_height_mm);
            if (length <= 0 || length >= static_cast<int>(sizeof(buffer))) {
                Uart1.print("nRF: command format failed\n");
                return;
            }
            NRF24L01P::str_touint8(
                etl::string_view(buffer, static_cast<std::size_t>(length)), NRF_Tx_Num);
        }

        if (!nRF.send(NRF_Tx_Num, NRF24L01P::PACKET_WIDTH)) {
            radioReady = false;
            recoveryTicks = 0U;
            Uart1.print("nRF: SPI send failed\n");
        }
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    });
    for(;;){

    }
}


TaskFunction_t RemoteControlFunc(){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); //
    uint16_t g_iAdcx[4]{};
    while(1){
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_iAdcx, sizeof(g_iAdcx) / sizeof(g_iAdcx[0]));
        float vlocity = V_Joy.get_converted_value(g_iAdcx[3]);
        float differ = D_Joy.get_converted_value(g_iAdcx[2]);
        float high =  H_Joy.get_converted_value(g_iAdcx[1]);
        float catroll = R_Joy.get_converted_value(g_iAdcx[0]);
        Uart1.print("{} {} {} {}\n",vlocity,differ,high,catroll);
//        Uart1.print("{} {} {} {}\n",g_iAdcx[3],g_iAdcx[2],g_iAdcx[1],g_iAdcx[0]);
        RemoteCommands.updateFromJoysticks(vlocity, differ, high, catroll);
        //绝对延时，保证周期稳定
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void RemoteDisplayFunc(void*)
{
    RemoteDisplay display;
    display.initialize();

    TickType_t last_wake_time = xTaskGetTickCount();
    constexpr TickType_t refresh_period = pdMS_TO_TICKS(100);
    while (true) {
        const auto command = RemoteCommands.snapshot();
        const RemoteDisplayState state{
            .speed = command.speed,
            .turn = command.turn,
            .leg_height_mm = command.leg_height_mm,
            .roll_degrees = command.roll_degrees,
            .leg_locked = command.leg_locked,
            .roll_locked = command.roll_locked,
        };
        display.render(state);
        vTaskDelayUntil(&last_wake_time, refresh_period);
    }
}

void GPIO_read(void*){
    TaskReactor reactor;
    Button legButton;
    Button rollButton;

    bool connected = true;
    connected &= reactor.connect(&legButton, &Button::signal_click, [] {
        if (RemoteCommands.toggleLegLock()) {
            Uart1.print("LEG: LOCK\n");
        } else {
            Uart1.print("LEG: LIVE\n");
        }
    });
    connected &= reactor.connect(&rollButton, &Button::signal_click, [] {
        if (RemoteCommands.toggleRollLock()) {
            Uart1.print("ROLL: LOCK\n");
        } else {
            Uart1.print("ROLL: LIVE\n");
        }
    });
    connected &= reactor.connect(&legButton, &Button::signal_long_press, [] {
        Uart1.print("PB0: long press, stack {} words\n", uxTaskGetStackHighWaterMark(nullptr));
    });
    connected &= reactor.connect(&rollButton, &Button::signal_long_press, [] {
        Uart1.print("PB1: long press, stack {} words\n", uxTaskGetStackHighWaterMark(nullptr));
    });

    if (!connected) {
        Uart1.print("Button reactor: connect failed\n");
    }

    reactor.taskLoop(pdMS_TO_TICKS(5), nullptr, [&legButton, &rollButton] {
        const std::uint32_t portState = GPIOB->IDR;
        const TickType_t now = xTaskGetTickCount();
        legButton.sample((portState & GPIO_PIN_0) == 0U, now);
        rollButton.sample((portState & GPIO_PIN_1) == 0U, now);
    });
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
    if (xTaskCreate((TaskFunction_t)RemoteControlFunc,
                    (const char*)"RemoteControl",
                    (uint16_t)256,
                    (void*)NULL,
                    (UBaseType_t)28,
                    (TaskHandle_t*)&Handle_ServoControlFunc) != pdPASS) {
        xReturn = pdFAIL;
    }
    if (xTaskCreate(GPIO_read,
                    (const char*)"GPIORead",
                    (uint16_t)512,
                    (void*)NULL,
                    (UBaseType_t)12,
                    (TaskHandle_t*)&Handle_GPIOReadFunc) != pdPASS) {
        xReturn = pdFAIL;
    }
    if (xTaskCreate(RemoteDisplayFunc,
                    "RemoteDisplay",
                    512,
                    nullptr,
                    10,
                    &Handle_RemoteDisplayFunc) != pdPASS) {
        xReturn = pdFAIL;
    }

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
