#pragma once

#include <cstdint>
#include <string_view>

#include "FreeRTOS.h"
#include "task.h"

#include "app_module.hpp"

namespace wl1::app_modules::detail {

class RtosTaskModule : public app::AppModule
{
public:
    RtosTaskModule(std::string_view module_name,
                   const char* task_name,
                   std::uint16_t stack_depth,
                   UBaseType_t priority,
                   TaskHandle_t& handle)
        : module_name_(module_name),
          task_name_(task_name),
          stack_depth_(stack_depth),
          priority_(priority),
          handle_(handle)
    {
    }

    [[nodiscard]] std::string_view name() const override
    {
        return module_name_;
    }

protected:
    virtual void run() = 0;

private:
    bool on_initialize() final
    {
        if (handle_ != nullptr)
        {
            return false;
        }

        return xTaskCreate(task_entry,
                           task_name_,
                           stack_depth_,
                           this,
                           priority_,
                           &handle_) == pdPASS;
    }

    bool on_deinitialize() final
    {
        if (handle_ != nullptr)
        {
            // These legacy tasks own drivers and timers on their task stacks. They can be
            // rolled back safely before scheduling starts, but must not be force-deleted at
            // runtime. A future runtime shutdown path must first make every task cooperative.
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
            {
                return false;
            }
            vTaskDelete(handle_);
            handle_ = nullptr;
        }
        return true;
    }

    static void task_entry(void* argument)
    {
        auto* self = static_cast<RtosTaskModule*>(argument);
        self->run();
        self->handle_ = nullptr;
        vTaskDelete(nullptr);
    }

    std::string_view module_name_;
    const char* task_name_;
    std::uint16_t stack_depth_;
    UBaseType_t priority_;
    TaskHandle_t& handle_;
};

} // namespace wl1::app_modules::detail
