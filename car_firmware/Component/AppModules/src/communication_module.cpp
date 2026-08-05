#include "app_modules.hpp"

#include <cstdint>
#include <functional>

#include "FreeRTOS.h"
#include "task.h"

#include "LegKinematics.hpp"
#include "TaskReactor.hpp"
#include "etl/format.h"
#include "etl/queue.h"
#include "etl/string.h"
#include "etl/string_view.h"
#include "etl/unordered_map.h"
#include "main.h"

#include "rtos_task_module.hpp"
#include "runtime.hpp"

namespace wl1::app_modules {
namespace {

using CommandHandler = std::function<void(etl::string_view)>;
using CommandMap = etl::unordered_map<etl::string_view, CommandHandler, 25, 57>;

class CommunicationModule final : public detail::RtosTaskModule
{
public:
    CommunicationModule()
        : RtosTaskModule("communication",
                         "LEDBlink",
                         2000,
                         28,
                         detail::runtime().communication_task)
    {
    }

private:
    void run() override
    {
        auto& runtime = detail::runtime();
        auto& control = runtime.control;
        auto& uart = runtime.uart;
        auto& radio = runtime.radio;

        TaskReactor reactor;
        TaskReactor::strCMD_t uart_command;
        std::uint8_t nrf_tx[NRF24L01P::PACKET_WIDTH]{};
        std::uint8_t nrf_rx[NRF24L01P::PACKET_WIDTH]{};
        etl::string_view nrf_rx_text{};

        CommandMap commands = {
            {"motor",
             [&runtime, &uart](etl::string_view arguments) {
                 std::uint16_t left = 0;
                 std::uint16_t right = 0;
                 if (TaskReactor::parseStrArg(arguments, left)
                     && TaskReactor::parseStrArg(arguments, right))
                 {
                     uart.print("motor: {}\t{}\n", left, right);
                     const std::uint32_t value =
                         (static_cast<std::uint32_t>(left) << 16U) | right;
                     xTaskNotify(runtime.motion_task, value, eSetValueWithOverwrite);
                 }
                 else
                 {
                     uart.print("Command \"motor\": Useless parameters\n");
                 }
             }},
            {"showimu",
             [&control](etl::string_view arguments) {
                 if (arguments.size() >= 2 && arguments[0] == '-')
                 {
                     if (arguments[1] == 'y')
                     {
                         control.show_imu_data = true;
                     }
                     else if (arguments[1] == 'n')
                     {
                         control.show_imu_data = false;
                     }
                 }
             }},
            {"showrpm",
             [&control](etl::string_view arguments) {
                 if (arguments.size() >= 2 && arguments[0] == '-')
                 {
                     if (arguments[1] == 'y')
                     {
                         control.show_motor_rpm = true;
                     }
                     else if (arguments[1] == 'n')
                     {
                         control.show_motor_rpm = false;
                     }
                 }
             }},
            {"anglepid",
             [&control](etl::string_view arguments) {
                 set_pid_value(arguments,
                               control.angle_kp,
                               control.angle_ki,
                               &control.angle_kd);
             }},
            {"velocitypid",
             [&control](etl::string_view arguments) {
                 set_pid_value(arguments,
                               control.velocity_kp,
                               control.velocity_ki,
                               &control.velocity_kd);
             }},
            {"differpid",
             [&control](etl::string_view arguments) {
                 set_pid_value(arguments,
                               control.differential_kp,
                               control.differential_ki,
                               &control.differential_kd);
             }},
            {"rollpid",
             [&control](etl::string_view arguments) {
                 set_pid_value(arguments, control.roll_kp, control.roll_ki, nullptr);
             }},
            {"nrfsend",
             [&radio, &nrf_tx](etl::string_view arguments) {
                 NRF24L01P::str_touint8(arguments, nrf_tx);
                 radio.send(nrf_tx, NRF24L01P::PACKET_WIDTH);
             }},
            {"nrfshow",
             [&control](etl::string_view arguments) {
                 configure_nrf_telemetry(arguments, control);
             }},
            {"legheight",
             [&control, &uart](etl::string_view arguments) {
                 float height = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, height))
                 {
                     float result_x = 0.0F;
                     const float result_degrees =
                         LegKinematics::getMotorAngleForHeight(height, &result_x);
                     const float bias =
                         ((-0.000155F * height + 0.03882F) * height - 3.001F) * height
                         + 83.25F;
                     uart.print("Servo angel: {:07.3f} {:07.3f} {:07.3f}\n",
                                result_degrees,
                                result_x,
                                bias);
                     control.target_height = height;
                 }
             }},
            {"target_roll",
             [&control](etl::string_view arguments) {
                 float target_roll = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, target_roll))
                 {
                     control.roll_target = target_roll;
                 }
             }},
            {"VandD",
             [&control](etl::string_view arguments) {
                 float velocity = 0.0F;
                 float differential = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, differential)
                     && TaskReactor::parseStrArg(arguments, velocity))
                 {
                     control.differential_target = differential;
                     control.velocity_target = velocity;
                 }
             }},
            {"R",
             [&control](etl::string_view arguments) {
                 float velocity = 0.0F;
                 float differential = 0.0F;
                 float height = 0.0F;
                 float roll = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, differential)
                     && TaskReactor::parseStrArg(arguments, velocity)
                     && TaskReactor::parseStrArg(arguments, roll)
                     && TaskReactor::parseStrArg(arguments, height))
                 {
                     control.differential_target = differential;
                     control.velocity_target = velocity;
                     control.roll_target = roll;
                     control.target_height = height;
                 }
             }},
            {"anglebias",
             [&control](etl::string_view arguments) {
                 float bias = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, bias))
                 {
                     control.angle_bias = bias;
                 }
             }},
        };

        etl::queue<etl::string<32>, 4> command_queue;

        radio.Init();
        radio.start_RxMode();
        uart.Start_DMAIT_Receive();

        reactor.connect(
            &uart,
            &LkUart<>::signal_RxComplete,
            [&command_queue](etl::string<128>& message) {
                if (command_queue.full())
                {
                    return;
                }
                if (message.size() > NRF24L01P::PACKET_WIDTH)
                {
                    command_queue.push(message.substr(0, NRF24L01P::PACKET_WIDTH));
                }
                else
                {
                    command_queue.push(message);
                }
            });

        reactor.connect(
            &radio,
            &NRF24L01P::signal_IRQEvent,
            [&radio, &uart, &nrf_rx, &nrf_rx_text, &command_queue](
                NRF24L01P::Status_t& status) {
                if (status.RX_DR)
                {
                    radio.tryReceive(nrf_rx);
                    NRF24L01P::uint8_tostr(nrf_rx_text, nrf_rx);
                    if (!command_queue.full())
                    {
                        command_queue.push(etl::string<32>(nrf_rx_text));
                    }
                }
                if (status.TX_DS)
                {
                    uart.print("nRF: send success\n");
                }
                if (status.MAX_RT)
                {
                    uart.print("nRF: send fail\n");
                }
            });

        reactor.taskLoop(
            pdMS_TO_TICKS(100),
            [&command_queue, &uart_command, &commands, &uart]() {
                while (!command_queue.empty())
                {
                    const auto command_text = command_queue.front();
                    command_queue.pop();
                    if (TaskReactor::parseStrCMD(command_text, uart_command))
                    {
                        const auto command = commands.find(uart_command.command);
                        if (command != commands.end())
                        {
                            command->second(uart_command.args);
                            continue;
                        }
                    }
                    uart.print("receive: {}\n", command_text);
                }
            },
            [&radio, &control, &nrf_tx]() {
                if (control.nrf_print_enabled)
                {
                    NRF24L01P::args_touint8s(nrf_tx, control.nrf_print_values);
                    radio.send(nrf_tx, NRF24L01P::PACKET_WIDTH);
                }
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            });
    }

    static void set_pid_value(etl::string_view arguments,
                              volatile float& proportional,
                              volatile float& integral,
                              volatile float* derivative)
    {
        if (arguments.size() < 2 || arguments[0] != '-')
        {
            return;
        }

        const char term = arguments[1];
        if (arguments.size() <= 3)
        {
            return;
        }
        arguments.remove_prefix(3);

        float value = 0.0F;
        if (!TaskReactor::parseStrArg(arguments, value))
        {
            return;
        }

        if (term == 'p')
        {
            proportional = value;
        }
        else if (term == 'i')
        {
            integral = value;
        }
        else if (term == 'd' && derivative != nullptr)
        {
            *derivative = value;
        }
    }

    static void configure_nrf_telemetry(etl::string_view arguments,
                                        detail::ControlState& control)
    {
        if (arguments.size() < 3 || arguments[0] != '-')
        {
            return;
        }
        if (arguments[1] == 'n' && arguments[2] == 'n')
        {
            control.nrf_print_enabled = false;
            return;
        }
        if (arguments[1] != 'm')
        {
            return;
        }

        const char source = arguments[2];
        if (arguments.size() <= 4)
        {
            return;
        }
        arguments.remove_prefix(4);

        std::uint8_t output_index = 0;
        if (!TaskReactor::parseStrArg(arguments, output_index))
        {
            return;
        }
        if (output_index > 3)
        {
            output_index = 3;
        }

        if (source == 'r')
        {
            control.nrf_print_values[output_index] = &control.euler_angles[0];
        }
        else if (source == 'p')
        {
            control.nrf_print_values[output_index] = &control.euler_angles[1];
        }
        else if (source == 'y')
        {
            control.nrf_print_values[output_index] = &control.euler_angles[2];
        }
        else
        {
            return;
        }
        control.nrf_print_enabled = true;
    }
};

} // namespace

app::AppModule& communication_module()
{
    static CommunicationModule module;
    return module;
}

} // namespace wl1::app_modules

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* spi)
{
    auto& radio = wl1::app_modules::detail::runtime().radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* spi)
{
    auto& radio = wl1::app_modules::detail::runtime().radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* spi)
{
    auto& radio = wl1::app_modules::detail::runtime().radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t pin)
{
    auto& radio = wl1::app_modules::detail::runtime().radio;
    if (pin == radio.getIRQGPIOPort())
    {
        radio.isrExtiHandler();
    }
}
