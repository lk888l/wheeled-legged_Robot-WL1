#include <array>
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
#include "adc.h"
#include "cpp_Interface.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

namespace {
constexpr TickType_t radio_period = pdMS_TO_TICKS(50U);
constexpr TickType_t radio_retry_period = pdMS_TO_TICKS(1000U);
constexpr TickType_t input_period = pdMS_TO_TICKS(50U);
constexpr TickType_t adc_timeout = pdMS_TO_TICKS(5U);
constexpr TickType_t button_period = pdMS_TO_TICKS(5U);
constexpr TickType_t display_period = pdMS_TO_TICKS(100U);

constexpr UBaseType_t radio_priority = tskIDLE_PRIORITY + 4U;
constexpr UBaseType_t input_priority = tskIDLE_PRIORITY + 3U;
constexpr UBaseType_t button_priority = tskIDLE_PRIORITY + 2U;
constexpr UBaseType_t display_priority = tskIDLE_PRIORITY + 1U;

constexpr std::uint16_t radio_stack_words = 768U;
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
    std::uint32_t max_retries = 0U;
    std::uint32_t driver_errors = 0U;
    std::uint32_t encoding_errors = 0U;
};

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

bool processRadioIrq(RadioCounters& counters)
{
    NRF24L01P::Status status;
    if (!radio.handleIrq(status)) {
        ++counters.driver_errors;
        debug_uart.print("radio: IRQ handling failed\n");
        return false;
    }

    if (status.tx_sent) {
        ++counters.sent;
    }
    if (status.max_retries) {
        ++counters.max_retries;
        if (counters.max_retries == 1U || counters.max_retries % 100U == 0U) {
            debug_uart.print(
                "radio: no ACK (count {})\n", counters.max_retries);
        }
    }
    return true;
}

bool transmitRemoteCommand(
    RemoteCommandCodec::Payload& payload, RadioCounters& counters)
{
    if (!RemoteCommandCodec::encode(remote_state.snapshot(), payload)) {
        ++counters.encoding_errors;
        debug_uart.print("radio: command encoding failed\n");
        return true;
    }

    if (!radio.send(payload.data(), static_cast<std::uint8_t>(payload.size()))) {
        ++counters.driver_errors;
        debug_uart.print("radio: SPI transmit failed\n");
        return false;
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    return true;
}

void radioTask(void*)
{
    // nRF24L01+ requires up to 100 ms after power-on before configuration.
    vTaskDelay(pdMS_TO_TICKS(100U));

    RadioCounters counters;
    RemoteCommandCodec::Payload payload{};
    std::uint32_t initialization_attempt = 0U;

    for (;;) {
        while (!initializeRadio(++initialization_attempt)) {
            vTaskDelay(radio_retry_period);
        }
        initialization_attempt = 0U;
        static_cast<void>(ulTaskNotifyTake(pdTRUE, 0U));

        TickType_t last_transmit = xTaskGetTickCount();
        bool driver_healthy = true;
        while (driver_healthy) {
            const TickType_t now = xTaskGetTickCount();
            const TickType_t elapsed = now - last_transmit;
            const TickType_t wait =
                elapsed >= radio_period ? 0U : radio_period - elapsed;

            if (ulTaskNotifyTake(pdTRUE, wait) > 0U) {
                driver_healthy = processRadioIrq(counters);
            }

            const TickType_t after_wait = xTaskGetTickCount();
            if (driver_healthy &&
                static_cast<TickType_t>(after_wait - last_transmit) >= radio_period) {
                last_transmit = after_wait;
                driver_healthy = transmitRemoteCommand(payload, counters);
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
