#pragma once

#include <string_view>

namespace wl1::app {

enum class ModuleState
{
    stopped,
    initialized,
    cleanup_failed,
};

class AppModule
{
public:
    virtual ~AppModule() = default;

    bool initialize()
    {
        if (state_ == ModuleState::initialized)
        {
            return true;
        }
        if (state_ == ModuleState::cleanup_failed)
        {
            return false;
        }

        if (on_initialize())
        {
            state_ = ModuleState::initialized;
            return true;
        }

        state_ = on_deinitialize() ? ModuleState::stopped : ModuleState::cleanup_failed;
        return false;
    }

    bool deinitialize()
    {
        if (state_ == ModuleState::stopped)
        {
            return true;
        }

        const bool cleaned = on_deinitialize();
        state_ = cleaned ? ModuleState::stopped : ModuleState::cleanup_failed;
        return cleaned;
    }

    [[nodiscard]] bool is_initialized() const
    {
        return state_ == ModuleState::initialized;
    }

    [[nodiscard]] ModuleState state() const
    {
        return state_;
    }

    [[nodiscard]] virtual std::string_view name() const = 0;
    virtual void process() {}

protected:
    AppModule() = default;

private:
    virtual bool on_initialize() = 0;
    virtual bool on_deinitialize() = 0;

    AppModule(const AppModule&) = delete;
    AppModule& operator=(const AppModule&) = delete;

    ModuleState state_ = ModuleState::stopped;
};

} // namespace wl1::app
