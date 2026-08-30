#include "app_modules.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

#include "FreeRTOS.h"
#include "task.h"

#include "LegKinematics.hpp"
#include "JumpController.hpp"
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

enum class CommandSource : std::uint8_t
{
    uart,
    radio,
};

struct QueuedCommand
{
    etl::string<32> text;
    CommandSource source = CommandSource::uart;
    TickType_t received_tick = 0;
};

struct PidTuningBounds
{
    float minimum_p;
    float maximum_p;
    float minimum_i;
    float maximum_i;
    float minimum_d;
    float maximum_d;
};

constexpr PidTuningBounds kAngleTuningBounds{40.0F, 120.0F, 0.0F, 1.0F, 30.0F, 100.0F};
constexpr PidTuningBounds kVelocityTuningBounds{0.0F, 0.15F, 0.0F, 0.03F, 0.0F, 0.10F};
constexpr PidTuningBounds kDifferentialTuningBounds{0.0F, 5.0F, 0.0F, 0.01F, 0.0F, 1.0F};
constexpr PidTuningBounds kRollTuningBounds{-1.0F, 1.0F, -1.0F, 0.0F, 0.0F, 0.0F};

bool isJumpActionState(wl1::control::JumpState state)
{
    return state == wl1::control::JumpState::preload
        || state == wl1::control::JumpState::thrust
        || state == wl1::control::JumpState::flight
        || state == wl1::control::JumpState::landing
        || state == wl1::control::JumpState::recover;
}

bool tuningLockedUnsafe(const detail::ControlState& control)
{
    const auto state = static_cast<wl1::control::JumpState>(control.jump_state);
    return control.jump_armed || isJumpActionState(state)
        || state == wl1::control::JumpState::fault;
}

template <typename Mutation>
bool applyTuningIfUnlocked(detail::ControlState& control, const Mutation& mutation)
{
    bool applied = false;
    taskENTER_CRITICAL();
    if (!tuningLockedUnsafe(control))
    {
        mutation();
        applied = true;
    }
    taskEXIT_CRITICAL();
    return applied;
}

enum class TuningWriteResult : std::uint8_t
{
    no_change,
    applied,
    locked,
};

bool heartbeatLedOn(bool radio_ready,
                    bool imu_valid,
                    bool wireless_command_valid,
                    bool jump_armed,
                    std::uint8_t raw_jump_state,
                    TickType_t now)
{
    constexpr TickType_t kSlotTicks = pdMS_TO_TICKS(100);
    static_assert(kSlotTicks > 0);
    const std::uint32_t slot = static_cast<std::uint32_t>(now / kSlotTicks) % 20U;

    if (!imu_valid)
    {
        // 200 ms on / 200 ms off: sensor/control recovery or fault.
        return (slot % 4U) < 2U;
    }

    const auto jump_state = static_cast<wl1::control::JumpState>(raw_jump_state);
    const bool jump_action = isJumpActionState(jump_state);
    if (!radio_ready)
    {
        // Three short pulses: nRF initialization/SPI fault.
        return slot < 2U || (slot >= 4U && slot < 6U)
            || (slot >= 8U && slot < 10U);
    }
    if (jump_state == wl1::control::JumpState::fault)
    {
        // Four short pulses: latched jump fault waiting for explicit disarm.
        return slot < 2U || (slot >= 4U && slot < 6U)
            || (slot >= 8U && slot < 10U) || (slot >= 12U && slot < 14U);
    }
    if (jump_action)
    {
        return true;
    }
    if (jump_armed)
    {
        // Double pulse: armed but no action active.
        return slot < 2U || (slot >= 4U && slot < 6U);
    }
    if (!wireless_command_valid)
    {
        // 500 ms on / 500 ms off: nRF is healthy but no fresh R frame.
        return (slot % 10U) < 5U;
    }
    // Normal heartbeat: one 200 ms pulse every two seconds.
    return slot < 2U;
}

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
        // PC13 LED is active-low on the target board. Start dark; the first
        // reactor timeout publishes the state pattern.
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

        TaskReactor reactor;
        TaskReactor::strCMD_t uart_command;
        std::uint8_t nrf_tx[NRF24L01P::PACKET_WIDTH]{};
        std::uint8_t nrf_rx[NRF24L01P::PACKET_WIDTH]{};
        etl::string_view nrf_rx_text{};
        etl::queue<QueuedCommand, 4> command_queue;
        CommandSource active_command_source = CommandSource::uart;
        TickType_t active_command_received_tick = 0;

        bool radio_ready = false;
        std::uint32_t radio_init_attempts = 0;
        std::uint32_t radio_irq_services = 0;
        std::uint32_t radio_rx_packets = 0;
        std::uint32_t radio_spi_errors = 0;
        std::uint32_t radio_queue_drops = 0;
        TickType_t last_radio_init_attempt = 0;

        NRF24L01P::StatusHandler radio_status_handler =
            [&radio,
             &uart,
             &nrf_rx,
             &nrf_rx_text,
             &command_queue,
             &radio_ready,
             &radio_rx_packets,
             &radio_spi_errors,
             &radio_queue_drops](NRF24L01P::Status_t& status) {
                if (status.IO_ERROR)
                {
                    ++radio_spi_errors;
                    radio_ready = false;
                    return false;
                }
                if (status.RX_DR)
                {
                    if (!radio.tryReceive(nrf_rx))
                    {
                        radio_ready = false;
                        return false;
                    }
                    ++radio_rx_packets;
                    NRF24L01P::uint8_tostr(nrf_rx_text, nrf_rx);
                    if (!command_queue.full())
                    {
                        command_queue.push(QueuedCommand{
                            etl::string<32>(nrf_rx_text),
                            CommandSource::radio,
                            xTaskGetTickCount()});
                    }
                    else
                    {
                        ++radio_queue_drops;
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
                return true;
            };

        const auto tuning_locked = [&control]() {
            taskENTER_CRITICAL();
            const bool locked = tuningLockedUnsafe(control);
            taskEXIT_CRITICAL();
            return locked;
        };

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
             [&control, &uart, &tuning_locked](etl::string_view arguments) {
                 if (tuning_locked())
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                     return;
                 }
                 if (arguments.size() >= 2 && arguments[0] == '-'
                     && arguments[1] == 'p')
                 {
                     if (arguments.size() <= 3)
                     {
                         return;
                     }
                     arguments.remove_prefix(3);
                     float value = 0.0F;
                     if (TaskReactor::parseStrArg(arguments, value) && std::isfinite(value))
                     {
                         const float clamped_value = std::clamp(
                             value,
                             kAngleTuningBounds.minimum_p,
                             kAngleTuningBounds.maximum_p);
                         if (!applyTuningIfUnlocked(control, [&control, clamped_value]() {
                                 control.angle_kp_override = clamped_value;
                                 control.angle_kp_override_enabled = true;
                             }))
                         {
                             uart.print("tuning: rejected while jump is armed/active\n");
                         }
                     }
                     return;
                 }
                 if (arguments == "auto")
                 {
                     if (!applyTuningIfUnlocked(control, [&control]() {
                             control.angle_kp_override_enabled = false;
                         }))
                     {
                         uart.print("tuning: rejected while jump is armed/active\n");
                     }
                     return;
                 }
                 if (set_pid_value(arguments,
                                   control,
                                   control.angle_kp,
                                   control.angle_ki,
                                   &control.angle_kd,
                                   kAngleTuningBounds)
                     == TuningWriteResult::locked)
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                 }
             }},
            {"velocitypid",
             [&control, &uart, &tuning_locked](etl::string_view arguments) {
                 if (tuning_locked())
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                     return;
                 }
                 if (set_pid_value(arguments,
                                   control,
                                   control.velocity_kp,
                                   control.velocity_ki,
                                   &control.velocity_kd,
                                   kVelocityTuningBounds)
                     == TuningWriteResult::locked)
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                 }
             }},
            {"differpid",
             [&control, &uart, &tuning_locked](etl::string_view arguments) {
                 if (tuning_locked())
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                     return;
                 }
                 if (set_pid_value(arguments,
                                   control,
                                   control.differential_kp,
                                   control.differential_ki,
                                   &control.differential_kd,
                                   kDifferentialTuningBounds)
                     == TuningWriteResult::locked)
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                 }
             }},
            {"rollpid",
             [&control, &uart, &tuning_locked](etl::string_view arguments) {
                 if (tuning_locked())
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                     return;
                 }
                 if (set_pid_value(arguments,
                                   control,
                                   control.roll_kp,
                                   control.roll_ki,
                                   nullptr,
                                   kRollTuningBounds)
                     == TuningWriteResult::locked)
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                 }
             }},
            {"nrfsend",
             [&radio, &nrf_tx, &radio_ready, &radio_spi_errors](
                 etl::string_view arguments) {
                 NRF24L01P::str_touint8(arguments, nrf_tx);
                 if (!radio_ready)
                 {
                     return;
                 }
                 if (!radio.send(nrf_tx, NRF24L01P::PACKET_WIDTH))
                 {
                     ++radio_spi_errors;
                     radio_ready = false;
                 }
             }},
            {"nrfshow",
             [&control](etl::string_view arguments) {
                 configure_nrf_telemetry(arguments, control);
             }},
            {"nrfstatus",
             [&uart,
              &radio,
              &radio_ready,
              &radio_init_attempts,
              &radio_irq_services,
              &radio_rx_packets,
              &radio_spi_errors,
              &radio_queue_drops](etl::string_view) {
                 uart.print("nRF ready={} irq={} init={} poll={} rx={} spierr={} drop={}\n",
                            radio_ready ? 1 : 0,
                            radio.isIRQAsserted() ? 1 : 0,
                            radio_init_attempts,
                            radio_irq_services,
                            radio_rx_packets,
                            radio_spi_errors,
                            radio_queue_drops);
             }},
            {"pidlevel",
             [&control, &uart](etl::string_view arguments) {
                 std::uint32_t level = 0;
                 if (!TaskReactor::parseStrArg(arguments, level) || level > 2U)
                 {
                     uart.print("pidlevel: use 0(gentle), 1(normal), or 2(sport)\n");
                     return;
                 }
                 taskENTER_CRITICAL();
                 control.requested_pid_level = static_cast<std::uint8_t>(level);
                 taskEXIT_CRITICAL();
                 uart.print("pidlevel: {}\n", level);
             }},
            {"controlstatus",
             [&control, &uart](etl::string_view) {
                 uart.print(
                     "ctl p={}/{} imu={} fail={}/{} rec={} arej={} sat={} cmd={} rf={} jump={}/{} servo={}/{} seq={}/{} hb={}\n",
                     static_cast<std::uint32_t>(control.active_pid_level),
                     static_cast<std::uint32_t>(control.requested_pid_level),
                     control.imu_valid ? 1 : 0,
                     control.imu_read_failures,
                     control.imu_consecutive_failures,
                     control.imu_recovery_samples,
                     control.imu_accel_rejected_streak,
                     control.imu_saturation_events,
                     control.remote_command_valid ? 1 : 0,
                     control.wireless_command_valid ? 1 : 0,
                     control.jump_armed ? 1 : 0,
                     static_cast<std::uint32_t>(control.jump_state),
                     control.servo_ready ? 1 : 0,
                     control.servo_command_complete ? 1 : 0,
                     control.servo_consumed_sequence,
                     control.servo_target_sequence,
                     xTaskGetTickCount() - control.servo_heartbeat_tick);
             }},
            {"jump",
             [&control, &uart](etl::string_view arguments) {
                 if (arguments == "arm")
                 {
                     taskENTER_CRITICAL();
                     control.jump_armed = true;
                     control.jump_request = false;
                     taskEXIT_CRITICAL();
                     uart.print("jump: armed (dry-run unless build option is enabled)\n");
                 }
                 else if (arguments == "trigger")
                 {
                     bool queued = false;
                     taskENTER_CRITICAL();
                     if (control.jump_armed)
                     {
                         control.jump_request = true;
                         queued = true;
                     }
                     taskEXIT_CRITICAL();
                     if (queued)
                     {
                         uart.print("jump: request queued\n");
                     }
                     else
                     {
                         uart.print("jump: rejected (not armed)\n");
                     }
                 }
                 else if (arguments == "disarm")
                 {
                     taskENTER_CRITICAL();
                     control.jump_armed = false;
                     control.jump_request = false;
                     control.jump_fault_clear_request = true;
                     taskEXIT_CRITICAL();
                     uart.print("jump: disarmed\n");
                 }
                 else if (arguments == "status")
                 {
                     uart.print("jump armed={} request={} state={} actuation={}\n",
                                control.jump_armed ? 1 : 0,
                                control.jump_request ? 1 : 0,
                                static_cast<std::uint32_t>(control.jump_state),
                                WL1_ENABLE_EXPERIMENTAL_JUMP);
                 }
                 else
                 {
                     uart.print("jump: use status|arm|trigger|disarm\n");
                 }
             }},
            {"legheight",
             [&control, &uart](etl::string_view arguments) {
                 float height = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, height) && std::isfinite(height))
                 {
                     height = std::clamp(height, 44.5F, 78.5F);
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
                 if (TaskReactor::parseStrArg(arguments, target_roll)
                     && std::isfinite(target_roll))
                 {
                     control.roll_target = std::clamp(target_roll, -18.0F, 18.0F);
                 }
             }},
            {"VandD",
             [&control](etl::string_view arguments) {
                 float velocity = 0.0F;
                 float differential = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, differential)
                     && TaskReactor::parseStrArg(arguments, velocity)
                     && std::isfinite(differential)
                     && std::isfinite(velocity))
                 {
                     taskENTER_CRITICAL();
                     control.differential_target = std::clamp(differential, -100.0F, 100.0F);
                     control.velocity_target = std::clamp(velocity, -100.0F, 100.0F);
                     taskEXIT_CRITICAL();
                 }
             }},
            {"R",
             [&control, &active_command_source, &active_command_received_tick](
                 etl::string_view arguments) {
                 float velocity = 0.0F;
                 float differential = 0.0F;
                 float height = 0.0F;
                 float roll = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, differential)
                     && TaskReactor::parseStrArg(arguments, velocity)
                     && TaskReactor::parseStrArg(arguments, roll)
                     && TaskReactor::parseStrArg(arguments, height)
                     && std::isfinite(differential)
                     && std::isfinite(velocity)
                     && std::isfinite(roll)
                     && std::isfinite(height))
                 {
                     differential = std::clamp(differential, -100.0F, 100.0F);
                     velocity = std::clamp(velocity, -100.0F, 100.0F);
                     roll = std::clamp(roll, -18.0F, 18.0F);
                     height = std::clamp(height, 44.5F, 78.5F);

                     std::uint32_t profile = 0;
                     const bool has_profile = TaskReactor::parseStrArg(arguments, profile);
                     std::uint32_t flags = 0;
                     const bool has_flags = has_profile
                         && TaskReactor::parseStrArg(arguments, flags);

                     taskENTER_CRITICAL();
                     control.differential_target = differential;
                     control.velocity_target = velocity;
                     control.roll_target = roll;
                     control.target_height = height;
                     if (has_profile)
                     {
                         control.requested_pid_level = static_cast<std::uint8_t>(
                             profile <= 2U ? profile : 1U);
                     }
                      if (has_flags)
                      {
                          const bool armed = (flags & 0x01U) != 0U;
                          const bool request_bit = (flags & 0x02U) != 0U;
                          if (!armed && control.jump_arm_bit_latched)
                          {
                              // A real arm-high -> arm-low transition is the
                              // acknowledgement required to clear a latched fault.
                              control.jump_fault_clear_request = true;
                          }
                          control.jump_armed = armed;
                         if (armed && request_bit && !control.jump_request_bit_latched)
                         {
                             control.jump_request = true;
                         }
                         if (!armed)
                         {
                             control.jump_request = false;
                          }
                          control.jump_arm_bit_latched = armed;
                          control.jump_request_bit_latched = request_bit;
                      }
                      else
                      {
                          // Legacy four-field frames never carry jump authority.
                          control.jump_armed = false;
                          control.jump_request = false;
                          control.jump_request_bit_latched = false;
                      }
                       control.last_remote_command_tick = active_command_received_tick;
                       control.remote_command_valid = true;
                       if (active_command_source == CommandSource::radio)
                       {
                           control.last_wireless_command_tick = active_command_received_tick;
                           control.wireless_command_valid = true;
                      }
                      taskEXIT_CRITICAL();
                 }
             }},
            {"anglebias",
             [&control, &uart, &tuning_locked](etl::string_view arguments) {
                 if (tuning_locked())
                 {
                     uart.print("tuning: rejected while jump is armed/active\n");
                     return;
                 }
                 if (arguments == "auto")
                 {
                     if (!applyTuningIfUnlocked(control, [&control]() {
                             control.angle_bias_override_enabled = false;
                         }))
                     {
                         uart.print("tuning: rejected while jump is armed/active\n");
                     }
                     return;
                 }
                 float bias = 0.0F;
                 if (TaskReactor::parseStrArg(arguments, bias) && std::isfinite(bias))
                 {
                     const float clamped_bias = std::clamp(bias, 5.0F, 20.0F);
                     if (!applyTuningIfUnlocked(control, [&control, clamped_bias]() {
                             control.angle_bias_override = clamped_bias;
                             control.angle_bias_override_enabled = true;
                         }))
                     {
                         uart.print("tuning: rejected while jump is armed/active\n");
                     }
                 }
             }},
        };

        const bool uart_connected = reactor.connect(
            &uart,
            &LkUart<>::signal_RxComplete,
             [&command_queue](etl::string<128>& message) {
                if (command_queue.full())
                {
                    return;
                }
                if (message.size() > NRF24L01P::PACKET_WIDTH)
                {
                    command_queue.push(QueuedCommand{
                        etl::string<32>(message.substr(0, NRF24L01P::PACKET_WIDTH)),
                         CommandSource::uart,
                         xTaskGetTickCount()});
                }
                else
                {
                    command_queue.push(QueuedCommand{
                        etl::string<32>(message),
                        CommandSource::uart,
                        xTaskGetTickCount()});
                }
            });

        const bool radio_connected = reactor.connect(
            &radio,
            &NRF24L01P::signal_IRQEvent,
            radio_status_handler);

        const bool uart_receive_started = uart_connected && uart.Start_DMAIT_Receive();
        if (!uart_receive_started)
        {
            uart.print("UART reactor/receive init failed\n");
        }
        if (!radio_connected)
        {
            uart.print("nRF reactor bind failed\n");
        }

        auto initialize_radio = [&]() {
            last_radio_init_attempt = xTaskGetTickCount();
            ++radio_init_attempts;
            radio_ready = radio.Init() && radio.start_RxMode();
            if (!radio_ready)
            {
                ++radio_spi_errors;
                uart.print("nRF init failed, retrying\n");
                return;
            }

            uart.print("nRF init success\n");
            // PA12 may already be low from a packet received before the EXTI
            // binding. Service it synchronously; no new falling edge is required.
            if (radio.isIRQAsserted())
            {
                ++radio_irq_services;
                if (!radio.signal_IRQEvent(radio_status_handler))
                {
                    radio_ready = false;
                }
            }
        };

        if (radio_connected)
        {
            // The radio's power-on reset interval runs after the scheduler starts,
            // so the balancing task is not stalled by a busy HAL delay.
            vTaskDelay(pdMS_TO_TICKS(100));
            initialize_radio();
        }

        reactor.taskLoop(
            pdMS_TO_TICKS(100),
            [&command_queue,
             &uart_command,
             &commands,
             &uart,
             &active_command_source,
             &active_command_received_tick,
             &radio_queue_drops]() {
                while (!command_queue.empty())
                {
                    const auto queued_command = command_queue.front();
                    command_queue.pop();
                    const auto& command_text = queued_command.text;
                    if (queued_command.source == CommandSource::radio
                        && (xTaskGetTickCount() - queued_command.received_tick)
                            > pdMS_TO_TICKS(250))
                    {
                        ++radio_queue_drops;
                        continue;
                    }
                    active_command_source = queued_command.source;
                    active_command_received_tick = queued_command.received_tick;
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
            [&radio,
             &control,
             &nrf_tx,
             &radio_status_handler,
             &initialize_radio,
             &radio_ready,
             &radio_connected,
              &last_radio_init_attempt,
              &radio_irq_services,
              &radio_spi_errors]() {
                 const TickType_t now = xTaskGetTickCount();
                 bool imu_valid = false;
                 bool wireless_command_valid = false;
                 bool jump_armed = false;
                 std::uint8_t jump_state = 0;
                 taskENTER_CRITICAL();
                 imu_valid = control.imu_valid;
                 wireless_command_valid = control.wireless_command_valid;
                 jump_armed = control.jump_armed;
                 jump_state = control.jump_state;
                 taskEXIT_CRITICAL();
                 // PC13 is the board's active-low LED. Absolute 100 ms slots
                 // avoid pattern drift when radio work delays this callback.
                 HAL_GPIO_WritePin(
                     GPIOC,
                     GPIO_PIN_13,
                     heartbeatLedOn(radio_ready,
                                    imu_valid,
                                    wireless_command_valid,
                                    jump_armed,
                                    jump_state,
                                    now)
                         ? GPIO_PIN_RESET
                         : GPIO_PIN_SET);
                 if (control.remote_command_valid
                     && (now - control.last_remote_command_tick) > pdMS_TO_TICKS(250))
                {
                    taskENTER_CRITICAL();
                    control.velocity_target = 0.0F;
                    control.differential_target = 0.0F;
                    control.roll_target = 0.0F;
                    control.remote_command_valid = false;
                    control.jump_armed = false;
                    control.jump_request = false;
                     control.jump_request_bit_latched = false;
                     taskEXIT_CRITICAL();
                 }
                 if (control.wireless_command_valid
                     && (now - control.last_wireless_command_tick) > pdMS_TO_TICKS(250))
                 {
                     taskENTER_CRITICAL();
                     control.wireless_command_valid = false;
                     control.jump_armed = false;
                     control.jump_request = false;
                     control.jump_request_bit_latched = false;
                     taskEXIT_CRITICAL();
                 }
                if (radio_connected && !radio_ready
                    && (now - last_radio_init_attempt) >= pdMS_TO_TICKS(1000))
                {
                    initialize_radio();
                }
                else if (radio_ready && radio.isIRQAsserted())
                {
                    // The nRF IRQ is level-held while STM32 EXTI is edge-triggered.
                    // This poll recovers any notification lost during startup or load.
                    ++radio_irq_services;
                    if (!radio.signal_IRQEvent(radio_status_handler))
                    {
                        radio_ready = false;
                    }
                }

                if (radio_ready && control.nrf_print_enabled)
                {
                    NRF24L01P::args_touint8s(nrf_tx, control.nrf_print_values);
                    if (!radio.send(nrf_tx, NRF24L01P::PACKET_WIDTH))
                    {
                        ++radio_spi_errors;
                        radio_ready = false;
                    }
                }
             });
    }

    static TuningWriteResult set_pid_value(etl::string_view arguments,
                                           detail::ControlState& control,
                                           volatile float& proportional,
                                           volatile float& integral,
                                           volatile float* derivative,
                                           const PidTuningBounds& bounds)
    {
        if (arguments.size() < 2 || arguments[0] != '-')
        {
            return TuningWriteResult::no_change;
        }

        const char term = arguments[1];
        if (arguments.size() <= 3)
        {
            return TuningWriteResult::no_change;
        }
        arguments.remove_prefix(3);

        float value = 0.0F;
        if (!TaskReactor::parseStrArg(arguments, value) || !std::isfinite(value))
        {
            return TuningWriteResult::no_change;
        }
        if (term == 'p')
        {
            const float clamped_value =
                std::clamp(value, bounds.minimum_p, bounds.maximum_p);
            return applyTuningIfUnlocked(control, [&proportional, clamped_value]() {
                       proportional = clamped_value;
                   })
                ? TuningWriteResult::applied
                : TuningWriteResult::locked;
        }
        if (term == 'i')
        {
            const float clamped_value =
                std::clamp(value, bounds.minimum_i, bounds.maximum_i);
            return applyTuningIfUnlocked(control, [&integral, clamped_value]() {
                       integral = clamped_value;
                   })
                ? TuningWriteResult::applied
                : TuningWriteResult::locked;
        }
        if (term == 'd' && derivative != nullptr)
        {
            const float clamped_value =
                std::clamp(value, bounds.minimum_d, bounds.maximum_d);
            return applyTuningIfUnlocked(control, [derivative, clamped_value]() {
                       *derivative = clamped_value;
                   })
                ? TuningWriteResult::applied
                : TuningWriteResult::locked;
        }
        return TuningWriteResult::no_change;
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
    auto* runtime = wl1::app_modules::detail::runtime_if_ready();
    if (runtime == nullptr)
    {
        return;
    }
    auto& radio = runtime->radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* spi)
{
    auto* runtime = wl1::app_modules::detail::runtime_if_ready();
    if (runtime == nullptr)
    {
        return;
    }
    auto& radio = runtime->radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* spi)
{
    auto* runtime = wl1::app_modules::detail::runtime_if_ready();
    if (runtime == nullptr)
    {
        return;
    }
    auto& radio = runtime->radio;
    if (spi == radio.getSPIHandle())
    {
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* spi)
{
    auto* runtime = wl1::app_modules::detail::runtime_if_ready();
    if (runtime == nullptr)
    {
        return;
    }
    auto& radio = runtime->radio;
    if (spi == radio.getSPIHandle())
    {
        // Release a task blocked on a failed DMA transaction. The transaction
        // itself still fails because HAL_SPI_GetError() is checked afterwards.
        radio.isrSpiDmaCompleteHandler();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t pin)
{
    auto* runtime = wl1::app_modules::detail::runtime_if_ready();
    if (runtime == nullptr)
    {
        return;
    }
    auto& radio = runtime->radio;
    if (pin == radio.getIRQGPIOPort())
    {
        radio.isrExtiHandler();
    }
}
