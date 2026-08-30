#include "runtime.hpp"

#include <atomic>

#include "spi.h"
#include "usart.h"

namespace wl1::app_modules::detail {

namespace {

constinit std::atomic<Runtime*> published_runtime{nullptr};
static_assert(std::atomic<Runtime*>::is_always_lock_free);

} // namespace

ControlState::ControlState()
    : nrf_print_values{&euler_angles[0], &euler_angles[1], &euler_angles[2], &angle_kp}
{
}

Runtime::Runtime()
    : uart(&huart1),
      radio(&hspi2, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_12, GPIOA, GPIO_PIN_12)
{
}

Runtime& runtime()
{
    static Runtime instance;
    published_runtime.store(&instance, std::memory_order_release);
    return instance;
}

Runtime* runtime_if_ready() noexcept
{
    return published_runtime.load(std::memory_order_acquire);
}

} // namespace wl1::app_modules::detail

namespace wl1::app_modules {

void initialize_runtime()
{
    static_cast<void>(detail::runtime());
}

void report_startup_result(bool success)
{
    if (success)
    {
        detail::runtime().uart.print("CPPMain: success\n");
    }
    else
    {
        detail::runtime().uart.print("CPPMain: fail\n");
    }
}

} // namespace wl1::app_modules
