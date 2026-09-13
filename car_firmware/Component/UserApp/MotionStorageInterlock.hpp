#pragma once

// All accesses use the same short FreeRTOS critical section as motor control.
// Even a save completed between two control ticks must restart the startup gate.
class MotionStorageInterlock {
public:
    bool begin(bool armed, int left_pwm, int right_pwm) noexcept
    {
        if (busy_ || armed || left_pwm != 0 || right_pwm != 0) return false;
        busy_ = true;
        reset_required_ = true;
        return true;
    }
    void finish() noexcept { busy_ = false; }
    bool busy() const noexcept { return busy_; }
    bool enabled() const noexcept { return enabled_; }
    bool canRun() const noexcept { return enabled_ && !busy_; }
    void setEnabled(bool enabled) noexcept
    {
        if (enabled_ != enabled) reset_required_ = true;
        enabled_ = enabled;
    }
    bool consumeReset() noexcept
    {
        const bool reset = reset_required_ || busy_ || !enabled_;
        reset_required_ = false;
        return reset;
    }
private:
    bool busy_ = false;
    bool reset_required_ = false;
    bool enabled_ = true; // Preserve automatic startup unless explicitly disabled.
};
