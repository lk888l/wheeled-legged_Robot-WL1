#include <array>
#include <algorithm>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "Button.hpp"
#include "Joystick.hpp"
#include "LkUart.hpp"
#include "NRF24L01P.hpp"
#include "RemoteCommandCodec.hpp"
#include "RemoteControlState.hpp"
#include "RemoteDisplay.hpp"
#include "SerialCommandQueue.hpp"
#include "adc.h"
#include "cpp_Interface.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

namespace {
constexpr TickType_t radio_period = pdMS_TO_TICKS(50U);
constexpr TickType_t radio_retry_period = pdMS_TO_TICKS(1000U);
constexpr TickType_t radio_tx_timeout = pdMS_TO_TICKS(30U);
constexpr TickType_t serial_idle_timeout = pdMS_TO_TICKS(50U);
constexpr TickType_t radio_poll_period = pdMS_TO_TICKS(5U);
constexpr TickType_t input_period = pdMS_TO_TICKS(50U);
constexpr TickType_t adc_timeout = pdMS_TO_TICKS(5U);
constexpr TickType_t button_period = pdMS_TO_TICKS(5U);
constexpr TickType_t display_period = pdMS_TO_TICKS(100U);

constexpr UBaseType_t radio_priority = tskIDLE_PRIORITY + 4U;
constexpr UBaseType_t input_priority = tskIDLE_PRIORITY + 3U;
constexpr UBaseType_t button_priority = tskIDLE_PRIORITY + 2U;
constexpr UBaseType_t display_priority = tskIDLE_PRIORITY + 1U;

constexpr std::uint16_t radio_stack_words = 1024U;
constexpr std::uint16_t input_stack_words = 256U;
constexpr std::uint16_t button_stack_words = 256U;
constexpr std::uint16_t display_stack_words = 512U;

LkUart debug_uart(&huart1);
NRF24L01P radio(
    &hspi2,
    GPIOA,
    GPIO_PIN_8,
    GPIOB,
    GPIO_PIN_12,
    GPIOA,
    GPIO_PIN_12);
RemoteControlState remote_state;

const Joystick speed_joystick(0U, 4095U, 2048U, 300U, -100.0F, 100.0F);
const Joystick turn_joystick(0U, 4095U, 2048U, 300U, -100.0F, 100.0F);
const Joystick roll_joystick(0U, 4095U, 2048U, 300U, -18.0F, 18.0F);
const Joystick leg_joystick(0U, 4095U, 2048U, 300U, 44.5F, 78.5F);

TaskHandle_t radio_task_handle = nullptr;
TaskHandle_t input_task_handle = nullptr;

struct RadioCounters {
    std::uint32_t sent = 0U;
    std::uint32_t received = 0U;
    std::uint32_t max_retries = 0U;
    std::uint32_t driver_errors = 0U;
    std::uint32_t encoding_errors = 0U;
};

struct Transmission {
    RemoteCommandCodec::Payload payload{};
    TickType_t started = 0U;
    bool pending = false;
    bool from_uart = false;
};

etl::string_view payloadText(const RemoteCommandCodec::Payload& payload)
{
    const auto end = std::find(payload.begin(), payload.end(), 0U);
    return {reinterpret_cast<const char*>(payload.data()),
            static_cast<std::size_t>(end - payload.begin())};
}

void notifyTaskFromIsr(TaskHandle_t task) noexcept
{
    if (task == nullptr) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

bool initializeRadio(std::uint32_t attempt)
{
    if (radio.initialize() && radio.startReceive()) {
        debug_uart.print("radio: ready\n");
        return true;
    }

    if (attempt == 1U || attempt % 5U == 0U) {
        debug_uart.print("radio: init failed (attempt {})\n", attempt);
    }
    return false;
}

bool processRadioIrq(RadioCounters& counters, Transmission& transmission)
{
    NRF24L01P::Status status;
    if (!radio.handleIrq(status)) {
        ++counters.driver_errors;
        debug_uart.print("radio: IRQ handling failed\n");
        return false;
    }

    if (status.tx_sent) {
        ++counters.sent;
        debug_uart.print("nRF: send success [{}]: {}\r\n",
                         transmission.from_uart ? "uart" : "joystick",
                         payloadText(transmission.payload));
        transmission.pending = false;
    }
    if (status.max_retries) {
        ++counters.max_retries;
        if (transmission.from_uart) {
            debug_uart.print("nRF: send fail [uart]: no ACK; {}\r\n",
                             payloadText(transmission.payload));
        } else if (counters.max_retries == 1U || counters.max_retries % 100U == 0U) {
            debug_uart.print(
                "radio: no ACK (count {})\n", counters.max_retries);
        }
        transmission.pending = false;
    }
    // RX FIFO has three slots. Bound the work so telemetry cannot starve control.
    for (std::uint8_t index = 0U; index < 3U; ++index) {
        bool available = false;
        RemoteCommandCodec::Payload received{};
        if (!radio.hasReceivedPayload(available)) {
            return false;
        }
        if (!available) {
            break;
        }
        if (!radio.receive(received.data())) {
            return false;
        }
        ++counters.received;
        debug_uart.print("receive: {}\r\n", payloadText(received));
    }
    return true;
}

bool sendPayload(const RemoteCommandCodec::Payload& payload, bool from_uart,
                 RadioCounters& counters, Transmission& transmission)
{
    transmission.payload = payload;
    transmission.from_uart = from_uart;
    if (from_uart) {
        debug_uart.print("uart: sending {}\r\n", payloadText(payload));
    }
    if (!radio.send(payload.data(), static_cast<std::uint8_t>(payload.size()))) {
        ++counters.driver_errors;
        debug_uart.print("radio: SPI transmit failed\r\n");
        return false;
    }
    transmission.started = xTaskGetTickCount();
    transmission.pending = true;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    return true;
}

void readSerialCommands(SerialCommandQueue& commands, TickType_t& last_receive,
                        std::uint32_t& receive_errors, std::uint32_t& queue_drops)
{
    const std::uint32_t errors = debug_uart.receiveErrors();
    if (errors != receive_errors) {
        commands.discardPartial();
        receive_errors = errors;
        debug_uart.print("uart: RX error/overflow (count {})\r\n", errors);
    }
    LkUart::ReceivedData received;
    // Bound each pass even if a PC continuously fills the DMA queue.
    for (std::uint8_t index = 0U; index < 8U && debug_uart.readReceived(received); ++index) {
        commands.append({received.data.data(), received.length});
        last_receive = xTaskGetTickCount();
    }
    if (static_cast<TickType_t>(xTaskGetTickCount() - last_receive) >= serial_idle_timeout) {
        commands.finish();
    }
    if (commands.droppedCommands() != queue_drops) {
        queue_drops = commands.droppedCommands();
        debug_uart.print("uart: command queue full (dropped {})\r\n", queue_drops);
    }
    static_cast<void>(debug_uart.startReceive(radio_task_handle));
}

bool processSerialCommands(SerialCommandQueue& commands, bool& joystick_enabled,
                           RadioCounters& counters, Transmission& transmission)
{
    SerialCommandQueue::Command command;
    if (!transmission.pending && commands.pop(command)) {
        switch (command.kind) {
        case SerialCommandQueue::Kind::transmit:
            return sendPayload(command.payload, true, counters, transmission);
        case SerialCommandQueue::Kind::joystick_on:
            joystick_enabled = true;
            debug_uart.print("joystick: on\r\n");
            break;
        case SerialCommandQueue::Kind::joystick_off:
            joystick_enabled = false;
            debug_uart.print("joystick: off\r\n");
            break;
        case SerialCommandQueue::Kind::help:
            debug_uart.print("commands: nrfsend <1..32 bytes> | R <turn> <speed> <roll> <height>\r\n");
            debug_uart.print("joystick on/off | status | help; CR/LF or 50 ms idle ends a command\r\n");
            break;
        case SerialCommandQueue::Kind::status:
            debug_uart.print("status: joystick={} sent={} no_ack={} received={} errors={}\r\n",
                             joystick_enabled ? "on" : "off", counters.sent,
                             counters.max_retries, counters.received, counters.driver_errors);
            debug_uart.print("uart: rx_errors={} dropped_logs={} stack={} words\r\n",
                             debug_uart.receiveErrors(), debug_uart.droppedMessages(),
                             uxTaskGetStackHighWaterMark(nullptr));
            break;
        case SerialCommandQueue::Kind::too_long:
            debug_uart.print("uart: command too long (radio payload max 32 bytes)\r\n");
            break;
        case SerialCommandQueue::Kind::invalid:
            debug_uart.print("uart: invalid command; use help or nrfsend <command>\r\n");
            break;
        }
    }
    return true;
}

void radioTask(void*)
{
    if (debug_uart.startReceive(radio_task_handle)) {
        debug_uart.print("uart: ready (115200 8N1); type help\r\n");
    }
    // nRF24L01+ requires up to 100 ms after power-on before configuration.
    vTaskDelay(pdMS_TO_TICKS(100U));

    RadioCounters counters;
    RemoteCommandCodec::Payload payload{};
    Transmission transmission;
    SerialCommandQueue serial_commands;
    bool joystick_enabled = true;
    TickType_t last_receive = xTaskGetTickCount();
    std::uint32_t receive_errors = 0U;
    std::uint32_t queue_drops = 0U;
    std::uint32_t initialization_attempt = 0U;

    for (;;) {
        while (!initializeRadio(++initialization_attempt)) {
            vTaskDelay(radio_retry_period);
        }
        initialization_attempt = 0U;
        transmission.pending = false;
        static_cast<void>(ulTaskNotifyTake(pdTRUE, 0U));

        TickType_t last_serial_service = xTaskGetTickCount() - radio_period;
        bool driver_healthy = true;
        while (driver_healthy) {
            static_cast<void>(ulTaskNotifyTake(pdTRUE, radio_poll_period));
            readSerialCommands(serial_commands, last_receive, receive_errors, queue_drops);

            // UART and radio share task notifications. Inspect the radio IRQ
            // level, and poll on a TX deadline to recover a missed EXTI edge.
            if (radio.irqAsserted() ||
                (transmission.pending &&
                 static_cast<TickType_t>(xTaskGetTickCount() - transmission.started) >= radio_tx_timeout)) {
                driver_healthy = processRadioIrq(counters, transmission);
            }
            if (driver_healthy && transmission.pending &&
                static_cast<TickType_t>(xTaskGetTickCount() - transmission.started) >= radio_tx_timeout) {
                ++counters.driver_errors;
                debug_uart.print("radio: TX result timeout\r\n");
                driver_healthy = false;
            }

            const TickType_t before_commands = xTaskGetTickCount();
            // Keep a burst of PC commands at the normal 20 Hz link rate, so
            // every result fits through the bounded UART logger as well.
            if (driver_healthy && !transmission.pending &&
                static_cast<TickType_t>(before_commands - last_serial_service) >= radio_period &&
                static_cast<TickType_t>(before_commands - transmission.started) >= radio_period) {
                last_serial_service = before_commands;
                driver_healthy = processSerialCommands(
                    serial_commands, joystick_enabled, counters, transmission);
            }

            const TickType_t after_wait = xTaskGetTickCount();
            if (driver_healthy && joystick_enabled && !transmission.pending &&
                static_cast<TickType_t>(after_wait - transmission.started) >= radio_period) {
                if (RemoteCommandCodec::encode(remote_state.snapshot(), payload)) {
                    driver_healthy = sendPayload(payload, false, counters, transmission);
                } else {
                    ++counters.encoding_errors;
                    debug_uart.print("radio: command encoding failed\r\n");
                }
            }
        }

        vTaskDelay(radio_retry_period);
    }
}

void inputTask(void*)
{
    std::array<std::uint16_t, 4U> adc_samples{};
    TickType_t last_wake = xTaskGetTickCount();
    std::uint32_t adc_errors = 0U;

    for (;;) {
        static_cast<void>(ulTaskNotifyTake(pdTRUE, 0U));
        const HAL_StatusTypeDef start_status = HAL_ADC_Start_DMA(
            &hadc1,
            reinterpret_cast<std::uint32_t*>(adc_samples.data()),
            static_cast<std::uint32_t>(adc_samples.size()));

        const bool conversion_complete =
            start_status == HAL_OK &&
            ulTaskNotifyTake(pdTRUE, adc_timeout) > 0U &&
            HAL_ADC_GetError(&hadc1) == HAL_ADC_ERROR_NONE;

        if (conversion_complete) {
            remote_state.updateFromJoysticks(
                speed_joystick.convert(adc_samples[3]),
                turn_joystick.convert(adc_samples[2]),
                leg_joystick.convert(adc_samples[1]),
                roll_joystick.convert(adc_samples[0]));
        } else {
            ++adc_errors;
            static_cast<void>(HAL_ADC_Stop_DMA(&hadc1));
            if (adc_errors == 1U || adc_errors % 100U == 0U) {
                debug_uart.print("input: ADC failure (count {})\n", adc_errors);
            }
        }

        vTaskDelayUntil(&last_wake, input_period);
    }
}

void handleLegButton(Button::Event event)
{
    if (event == Button::Event::clicked) {
        if (remote_state.toggleLegLock()) {
            debug_uart.print("leg: locked\n");
        } else {
            debug_uart.print("leg: live\n");
        }
    } else if (event == Button::Event::long_pressed) {
        debug_uart.print(
            "buttons: stack {} words\n", uxTaskGetStackHighWaterMark(nullptr));
    }
}

void handleRollButton(Button::Event event)
{
    if (event == Button::Event::clicked) {
        if (remote_state.toggleRollLock()) {
            debug_uart.print("roll: locked\n");
        } else {
            debug_uart.print("roll: live\n");
        }
    } else if (event == Button::Event::long_pressed) {
        debug_uart.print(
            "buttons: stack {} words\n", uxTaskGetStackHighWaterMark(nullptr));
    }
}

void buttonTask(void*)
{
    Button leg_button;
    Button roll_button;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const std::uint32_t port_state = GPIOB->IDR;
        const TickType_t now = xTaskGetTickCount();
        handleLegButton(leg_button.sample((port_state & GPIO_PIN_0) == 0U, now));
        handleRollButton(roll_button.sample((port_state & GPIO_PIN_1) == 0U, now));
        vTaskDelayUntil(&last_wake, button_period);
    }
}

void displayTask(void*)
{
    RemoteDisplay display;
    static_cast<void>(display.initialize());
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const RemoteControlSnapshot command = remote_state.snapshot();
        display.render(RemoteDisplayState{
            .speed = command.speed,
            .turn = command.turn,
            .leg_height_mm = command.leg_height_mm,
            .roll_degrees = command.roll_degrees,
            .leg_locked = command.leg_locked,
            .roll_locked = command.roll_locked,
        });
        vTaskDelayUntil(&last_wake, display_period);
    }
}

bool createTask(
    TaskFunction_t function,
    const char* name,
    std::uint16_t stack_words,
    UBaseType_t priority,
    TaskHandle_t* handle = nullptr)
{
    return xTaskCreate(
               function, name, stack_words, nullptr, priority, handle) == pdPASS;
}
} // namespace

void CPP_Main()
{
    const bool radio_created = createTask(
        radioTask,
        "Radio",
        radio_stack_words,
        radio_priority,
        &radio_task_handle);
    const bool input_created = createTask(
        inputTask,
        "Input",
        input_stack_words,
        input_priority,
        &input_task_handle);
    const bool buttons_created = createTask(
        buttonTask, "Buttons", button_stack_words, button_priority);
    const bool display_created = createTask(
        displayTask, "Display", display_stack_words, display_priority);

    configASSERT(radio_created && input_created && buttons_created && display_created);
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* spi)
{
    if (spi == radio.spiHandle()) {
        radio.onSpiTransferCompleteFromIsr();
    }
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* spi)
{
    if (spi == radio.spiHandle()) {
        radio.onSpiTransferCompleteFromIsr();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* spi)
{
    if (spi == radio.spiHandle()) {
        radio.onSpiTransferCompleteFromIsr();
    }
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* spi)
{
    if (spi == radio.spiHandle()) {
        radio.onSpiTransferCompleteFromIsr();
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(std::uint16_t pin)
{
    if (pin == radio.irqPin()) {
        notifyTaskFromIsr(radio_task_handle);
    }
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc)
{
    if (adc == &hadc1) {
        notifyTaskFromIsr(input_task_handle);
    }
}

extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* adc)
{
    if (adc == &hadc1) {
        notifyTaskFromIsr(input_task_handle);
    }
}
