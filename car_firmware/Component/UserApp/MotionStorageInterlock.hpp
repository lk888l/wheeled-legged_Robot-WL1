#pragma once

// All accesses use the same short FreeRTOS critical section as motor control.
// Appending blank Flash words leaves control running. Erase-capable maintenance
// alone requires zero output and restarts the startup gate.
class MotionStorageInterlock {
public:
    bool begin(bool armed, int left_pwm, int right_pwm, bool exclusive = true) noexcept
    {
        if (busy_ || (exclusive && (armed || left_pwm != 0 || right_pwm != 0))) return false;
        busy_ = true;
        blocks_control_ = exclusive;
        if (exclusive) reset_required_ = true;
        return true;
    }
    void finish() noexcept { busy_ = blocks_control_ = false; }
    bool busy() const noexcept { return busy_; }
    bool blocksControl() const noexcept { return blocks_control_; }
    bool enabled() const noexcept { return enabled_; }
    bool canRun() const noexcept { return enabled_ && !blocks_control_; }
    void requestReset() noexcept { reset_required_ = true; }
    void setEnabled(bool enabled) noexcept
    {
        if (enabled_ != enabled) reset_required_ = true;
        enabled_ = enabled;
    }
    bool consumeReset() noexcept
    {
        const bool reset = reset_required_ || blocks_control_ || !enabled_;
        reset_required_ = false;
        return reset;
    }
private:
    bool busy_ = false;
    bool blocks_control_ = false;
    bool reset_required_ = false;
    bool enabled_ = true; // Preserve automatic startup unless explicitly disabled.
};
