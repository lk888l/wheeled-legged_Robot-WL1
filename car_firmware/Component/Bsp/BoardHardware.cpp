#include "BoardHardware.hpp"

#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

namespace {

NRF24L01P* g_radio_for_isr = nullptr;

template <bool (bsp::BoardHardware::*Initialize)()>
bool initialize_adapter(void* context)
{
    if (context == nullptr) {
        return false;
    }
    return (static_cast<bsp::BoardHardware*>(context)->*Initialize)();
}

} // namespace

namespace bsp {

BoardHardware::BoardHardware()
    : command_uart_(&huart1),
      imu_(&hi2c1,
           {MPU6050::GyroRange_t::G1000,
            MPU6050::AccRange_t::A4,
            static_cast<uint16_t>(MPU6050::ms_toHZ(10)),
            {0, 0, 0}}),
      left_encoder_(&htim2, HallEncoder::InitConfig_t{7, 50, 4, 50}),
      right_encoder_(&htim3, HallEncoder::InitConfig_t{7, 50, 4, 50}),
      wheel_motor_(TB6612::InitConfig_t{
          .Htim = &htim1,
          .AChannel = TIM_CHANNEL_1,
          .BChannel = TIM_CHANNEL_2,
          .A1GPIO_Port = AIN1_GPIO_Port,
          .A1GPIO_Pin = AIN1_Pin,
          .A2GPIO_Port = AIN2_GPIO_Port,
          .A2GPIO_Pin = AIN2_Pin,
          .B1GPIO_Port = BIN1_GPIO_Port,
          .B1GPIO_Pin = BIN1_Pin,
          .B2GPIO_Port = BIN2_GPIO_Port,
          .B2GPIO_Pin = BIN2_Pin}),
      left_servo_(&htim9,
                  TIM_CHANNEL_1,
                  Servo::PhysicalToPulse(0.0F),
                  Servo::PhysicalToPulse(180.0F),
                  180.0F),
      right_servo_(&htim9,
                   TIM_CHANNEL_2,
                   Servo::PhysicalToPulse(169.0F),
                   Servo::PhysicalToPulse(11.0F),
                   180.0F),
      radio_(&hspi2,
             GPIOA,
             GPIO_PIN_4,
             GPIOB,
             GPIO_PIN_12,
             GPIOA,
             GPIO_PIN_12)
{
    imu_.setGyroOffset(2.5, 0.7, 0.9);
    wheel_motor_.setDirection_Cfg(static_cast<uint8_t>(TB6612::OutPort::B),
                                  TB6612::Direction::Negative);
    wheel_motor_.setA_DeadZone(50);
    wheel_motor_.setB_DeadZone(50);
    left_servo_.setLimit(0.0F, 50.0F);
    right_servo_.setLimit(0.0F, 50.0F);
    g_radio_for_isr = &radio_;
}

bool BoardHardware::initialize_command_uart()
{
    return command_uart_.Start_DMAIT_Receive();
}

bool BoardHardware::initialize_imu()
{
    return imu_.Init();
}

bool BoardHardware::initialize_left_encoder()
{
    if (!left_encoder_.Init()) {
        return false;
    }
    left_encoder_.clearCounter();
    return true;
}

bool BoardHardware::initialize_right_encoder()
{
    if (!right_encoder_.Init()) {
        return false;
    }
    right_encoder_.clearCounter();
    return true;
}

bool BoardHardware::initialize_wheel_motor()
{
    const bool initialized = wheel_motor_.Init();
    wheel_motor_.forceStop();
    return initialized;
}

bool BoardHardware::initialize_left_servo()
{
    return left_servo_.Init();
}

bool BoardHardware::initialize_right_servo()
{
    return right_servo_.Init();
}

bool BoardHardware::initialize_radio()
{
    return radio_.Init() && radio_.start_RxMode();
}

void BoardHardware::force_safe_outputs()
{
    wheel_motor_.forceStop();
    left_servo_.stop();
    right_servo_.stop();
}

HardwareFactory::InitializationPlan HardwareFactory::create_initialization_plan(
    BoardHardware& hardware)
{
    return {{
        {module_id(HardwareModuleId::command_uart),
         "command-uart",
         initialize_adapter<&BoardHardware::initialize_command_uart>,
         &hardware},
        {module_id(HardwareModuleId::imu),
         "imu-mpu6050",
         initialize_adapter<&BoardHardware::initialize_imu>,
         &hardware},
        {module_id(HardwareModuleId::left_encoder),
         "left-encoder",
         initialize_adapter<&BoardHardware::initialize_left_encoder>,
         &hardware},
        {module_id(HardwareModuleId::right_encoder),
         "right-encoder",
         initialize_adapter<&BoardHardware::initialize_right_encoder>,
         &hardware},
        {module_id(HardwareModuleId::wheel_motor),
         "wheel-motor",
         initialize_adapter<&BoardHardware::initialize_wheel_motor>,
         &hardware},
        {module_id(HardwareModuleId::left_servo),
         "left-servo",
         initialize_adapter<&BoardHardware::initialize_left_servo>,
         &hardware},
        {module_id(HardwareModuleId::right_servo),
         "right-servo",
         initialize_adapter<&BoardHardware::initialize_right_servo>,
         &hardware},
        {module_id(HardwareModuleId::radio),
         "radio-nrf24",
         initialize_adapter<&BoardHardware::initialize_radio>,
         &hardware},
    }};
}

} // namespace bsp

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (g_radio_for_isr != nullptr && hspi == g_radio_for_isr->getSPIHandle()) {
        g_radio_for_isr->isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (g_radio_for_isr != nullptr && hspi == g_radio_for_isr->getSPIHandle()) {
        g_radio_for_isr->isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (g_radio_for_isr != nullptr && hspi == g_radio_for_isr->getSPIHandle()) {
        g_radio_for_isr->isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (g_radio_for_isr != nullptr && gpio_pin == g_radio_for_isr->getIRQGPIOPort()) {
        g_radio_for_isr->isrExtiHandler();
    }
}
