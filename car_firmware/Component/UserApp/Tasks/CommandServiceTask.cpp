#include "CommandServiceTask.hpp"

#include <algorithm>
#include "BoardHardware.hpp"
#include "ButtonTask.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "TaskReactor.hpp"
#include "TelemetryFrame.hpp"
#include "CtrlAlgorithm/LegKinematics.hpp"

namespace app {
namespace {

bool parse_value(etl::string_view args, float& value)
{
    return text_command::parse_argument(args, value) && args.empty();
}

bool update_gains(etl::string_view args, PidGains& gains, bool allow_derivative)
{
    text_command::ParsedCommand option;
    float value{};
    if (!text_command::parse(args, option) || !parse_value(option.args, value)) {
        return false;
    }
    if (option.command == "-p") { gains.kp = value; }
    else if (option.command == "-i") { gains.ki = value; }
    else if (option.command == "-d" && allow_derivative) { gains.kd = value; }
    else { return false; }
    return true;
}

} // namespace

void CommandServiceTask::run()
{
    TaskReactor reactor;
    if (status_.hardware_ready(bsp::module_id(bsp::HardwareModuleId::command_uart))) {
        const bool connected = reactor.connect(&board_.command_uart(),
            &LkUart<>::signal_RxComplete,
            [this](etl::string<128U>& bytes) {
                // An empty event reports a receive error or dropped UART data.
                if (bytes.empty()) { uart_framer_.discard_partial(); return; }
                const auto tick = xTaskGetTickCount();
                uart_framer_.expire(tick, task_config::uart_frame_timeout);
                uart_framer_.feed(bytes, [this](etl::string_view frame) { enqueue_command(frame); }, tick);
            });
        configASSERT(connected);
    }
    if (status_.hardware_ready(bsp::module_id(bsp::HardwareModuleId::radio))) {
        const bool connected = reactor.connect(&board_.radio(),
            &NRF24L01P::signal_IRQEvent,
            [this](NRF24L01P::Status_t& radio_status) {
                if (radio_status.RX_DR && board_.radio().tryReceive(radio_rx_.data())) {
                    etl::string_view frame;
                    NRF24L01P::uint8_tostr(frame, radio_rx_.data());
                    enqueue_command(frame);
                }
                if (radio_status.TX_DS) {
                    board_.command_uart().print("nRF: send success\n");
                }
                if (radio_status.MAX_RT) {
                    board_.command_uart().print("nRF: send fail\n");
                }
            });
        configASSERT(connected);
    }
    reactor.taskLoop(pdMS_TO_TICKS(100U), [this] {
        // A bounded batch prevents continuous input from extending this turn.
        for (size_t count = 0U; count < 4U && !commands_.empty(); ++count) {
            const auto frame = commands_.front();
            commands_.pop();
            process_command(frame);
        }
    }, [this] { service_periodic(); });
}

void CommandServiceTask::enqueue_command(etl::string_view frame)
{
    if (commands_.full()) {
        board_.command_uart().print("command dropped: queue full\n");
        return;
    }
    // Never execute a truncated control request as if it were a valid frame.
    if (frame.size() > 32U) {
        board_.command_uart().print("command dropped: frame too long\n");
        return;
    }
    commands_.push(etl::string<32U>(frame));
}

void CommandServiceTask::process_command(etl::string_view frame)
{
    text_command::ParsedCommand parsed;
    auto& uart = board_.command_uart();
    if (!text_command::parse(frame, parsed)) { return; }
    const auto name = parsed.command;
    auto args = parsed.args;

    if (name == "ping") {
        uart.print("pong state={} control={}\n", system_state_name(status_.state()),
                   status_.control_enabled() ? "on" : "off");
        return;
    }
    if (name == "status") {
        const auto feedback = control_.feedback();
        uart.print("status={} control={} hw_fail={} task_fail={} armed={} imu={}\n",
                   system_state_name(status_.state()),
                   status_.control_enabled() ? "on" : "off",
                   status_.hardware_failed_mask(), status_.task_failed_mask(),
                   feedback.armed ? 1 : 0, feedback.imu_valid ? 1 : 0);
        return;
    }
    if (name == "controlstate") {
        const auto f = control_.feedback();
        uart.print("armed={} imu={} pitch={:.3f} pwm={},{}\n", f.armed ? 1 : 0,
                   f.imu_valid ? 1 : 0, f.pitch_error, f.left_pwm, f.right_pwm);
        uart.print("loops={} tick={} gap={} missed={}\n", f.loop_count, f.sample_tick,
                   f.max_sample_gap_ticks, f.deadline_misses);
        uart.print("remote_timeout={} velocity={:.1f} turn={:.1f} roll={:.1f}\n",
                   f.remote_timed_out ? 1 : 0, f.velocity_target, f.difference_target, f.roll_target);
        return;
    }
    if (name == "button") {
        drain_button_events();
        uart.print("button A0={} click={} double={} long={} drop={} gap={} tick\n",
                   button_.is_running() ? "on" : "off", button_counts_[0],
                   button_counts_[1], button_counts_[2], events_.dropped(),
                   button_.max_sample_gap_ticks());
        return;
    }
    if (name == "motor") {
        // The active PID controller never consumed the old raw-PWM mailbox.
        // Do not acknowledge an actuator command that will not be applied.
        uart.print("motor rejected: raw PWM is unavailable in PID control mode\n");
        return;
    }
    if (name == "nrfsend") {
        if (!status_.hardware_ready(bsp::module_id(bsp::HardwareModuleId::radio))) {
            uart.print("nrfsend rejected: radio unavailable\n");
        } else {
            NRF24L01P::str_touint8(args, radio_tx_.data());
            (void)board_.radio().send(radio_tx_.data(), radio_tx_.size());
        }
        return;
    }
    if (name == "nrfshow") {
        text_command::ParsedCommand option;
        uint8_t index{};
        if (text_command::parse(args, option)) {
            if (option.command == "-nn" && option.args.empty()) {
                telemetry_enabled_ = false;
                return;
            }
            if (text_command::parse_argument(option.args, index) && option.args.empty()) {
                index = std::min<uint8_t>(index, 3U);
                if (option.command == "-mr") { telemetry_sources_[index] = 0U; }
                else if (option.command == "-mp") { telemetry_sources_[index] = 1U; }
                else if (option.command == "-my") { telemetry_sources_[index] = 2U; }
                else { uart.print("nrfshow: invalid option\n"); return; }
                telemetry_enabled_ = true;
                return;
            }
        }
        uart.print("nrfshow: invalid parameters\n");
        return;
    }

    auto parameters = control_.parameters();
    if ((name == "R" || name == "VandD" || name == "target_roll") &&
        parameters.motion_command_received &&
        static_cast<TickType_t>(xTaskGetTickCount() - parameters.motion_command_tick) >=
            task_config::remote_timeout) {
        // A new roll-only command cannot revive an expired driving target.
        parameters.velocity_target = parameters.difference_target = parameters.roll_target = 0.0F;
    }
    if (args.empty()) {
        const PidGains* gains = name == "anglepid" ? &parameters.angle :
            name == "velocitypid" ? &parameters.velocity :
            name == "differpid" ? &parameters.difference :
            name == "rollpid" ? &parameters.roll : nullptr;
        if (gains != nullptr) {
            uart.print("{} p={:.4f} i={:.4f} d={:.4f}\n", name, gains->kp, gains->ki, gains->kd);
            if (name == "anglepid") {
                uart.print("anglepid mode={} effective_p={:.4f}\n",
                    parameters.angle_kp_auto ? "auto" : "manual", control_.feedback().angle_kp);
            }
            return;
        }
        if (name == "anglebias") {
            uart.print("anglebias base={:.4f} effective={:.4f}\n",
                       parameters.angle_bias, control_.feedback().angle_bias);
            return;
        }
    }
    bool accepted = false;
    if (name == "showimu" || name == "showrpm") {
        accepted = args == "-y" || args == "-n";
        if (name == "showimu") { parameters.show_imu = args == "-y"; }
        else { parameters.show_rpm = args == "-y"; }
    } else if (name == "anglepid") {
        if (args == "-auto") {
            parameters.angle_kp_auto = true;
            accepted = true;
        } else {
            accepted = update_gains(args, parameters.angle, true);
            text_command::ParsedCommand option;
            if (accepted && text_command::parse(args, option) && option.command == "-p") {
                parameters.angle_kp_auto = false;
            }
        }
    } else if (name == "velocitypid") {
        accepted = update_gains(args, parameters.velocity, true);
    } else if (name == "differpid") {
        accepted = update_gains(args, parameters.difference, true);
    } else if (name == "rollpid") {
        accepted = update_gains(args, parameters.roll, true);
    } else if (name == "anglebias") {
        // This is the minimum-height baseline; MotionControl adds height compensation.
        accepted = parse_value(args, parameters.angle_bias);
    } else if (name == "legheight") {
        accepted = parse_value(args, parameters.leg_height);
        if (accepted) {
            const float height = parameters.leg_height;
            float result_x{};
            const float degrees = LegKinematics::getMotorAngleForHeight(height, &result_x);
            const float bias = ((-0.000155F * height + 0.03882F) * height - 3.001F) * height + 83.25F;
            uart.print("Servo angel: {:07.3f} {:07.3f} {:07.3f}\n", degrees, result_x, bias);
        }
    } else if (name == "target_roll") {
        accepted = parse_value(args, parameters.roll_target);
    } else if (name == "VandD" || name == "R") {
        accepted = text_command::parse_argument(args, parameters.difference_target) &&
                   text_command::parse_argument(args, parameters.velocity_target);
        if (accepted && name == "R") {
            accepted = text_command::parse_argument(args, parameters.roll_target) &&
                       text_command::parse_argument(args, parameters.leg_height);
        }
        accepted = accepted && args.empty();
    } else {
        uart.print("receive: {}\n", frame);
        return;
    }

    if (accepted) {
        if (name == "R" || name == "VandD" || name == "target_roll") {
            parameters.motion_command_tick = xTaskGetTickCount();
            parameters.motion_command_received = true;
            // Match the physical remote and mini-program limits on the MCU too.
            parameters.velocity_target = std::clamp(parameters.velocity_target, -100.0F, 100.0F);
            parameters.difference_target = std::clamp(parameters.difference_target, -100.0F, 100.0F);
            parameters.roll_target = std::clamp(parameters.roll_target, -18.0F, 18.0F);
        }
        if (name == "R" || name == "legheight") {
            parameters.leg_height = BalanceCompensation::clampLegHeight(parameters.leg_height);
        }
        // One short, coherent publication: R cannot expose half-updated targets.
        control_.set_parameters(parameters);
    } else {
        uart.print("Command \"{}\": invalid parameters\n", name);
    }
}

void CommandServiceTask::service_periodic()
{
    if (status_.hardware_ready(bsp::module_id(bsp::HardwareModuleId::command_uart))) {
        board_.command_uart().service_receive();
    }
    drain_button_events();
    if (telemetry_enabled_ &&
        status_.hardware_ready(bsp::module_id(bsp::HardwareModuleId::radio))) {
        const auto feedback = control_.feedback();
        std::array<float, 4U> values{};
        for (size_t i = 0U; i < values.size(); ++i) {
            values[i] = telemetry_sources_[i] < 3U
                ? feedback.euler[telemetry_sources_[i]] : feedback.angle_kp;
        }
        if (radio_frame::encode_telemetry(values, radio_tx_)) {
            (void)board_.radio().send(radio_tx_.data(), radio_tx_.size());
        }
    }
}

void CommandServiceTask::drain_button_events()
{
    ButtonEvent event;
    for (uint32_t i = 0U; i < ButtonEventQueue::capacity && events_.try_pop(event); ++i) {
        handle_button_event(event);
    }
}

void CommandServiceTask::handle_button_event(ButtonEvent event)
{
    last_button_ = event;
    // Business integration point. Keep handlers bounded and nonblocking; send
    // slow work to a lower-priority owner. No actuator action is assigned yet.
    switch (event.type) {
    case Button::Event::click: ++button_counts_[0]; break;
    case Button::Event::double_click: ++button_counts_[1]; break;
    case Button::Event::long_press: ++button_counts_[2]; break;
    case Button::Event::none: break;
    }
}

} // namespace app
