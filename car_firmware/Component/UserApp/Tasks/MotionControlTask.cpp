#include "MotionControlTask.hpp"

#include <algorithm>
#include <cmath>
#include "BoardHardware.hpp"
#include "ControlState.hpp"
#include "RuntimeStatus.hpp"
#include "CtrlAlgorithm/BalanceStartupGate.hpp"
#include "CtrlAlgorithm/PID.hpp"

namespace app {

void MotionControlTask::run()
{
    TickType_t last_wake;
    const TickType_t period = task_config::motion_period;
    LegTargets legs{};
    ControlFeedback feedback{};
    BalanceStartupGate startup_gate;
    uint8_t velocity_loop_count = 0;
    uint8_t imu_read_failures = 0U;
    //  PID
    PID angle_pid(70.0f,0,51.0f,-1000,1000,-100,100);
    PID velocity_pid(0.04,0.006,0,-10,10,-100,100);
    PID difference_pid(0,0,0,-500,500,-100,100);
    PID roll_pid(0,0,0,-78,78,-100,100);
    float difference_rpm{}, angle_target{}, difference_pwm{};
    float last_target_roll{};
    uint32_t last_motion_command_tick{};
    //sensor
    auto& imu = board_.imu();
    MPU6050::EulerAngle angle{};
    double gyro[3]{};
    auto& left_encoder = board_.left_encoder();
    auto& right_encoder = board_.right_encoder();
    auto& wheel_motor = board_.wheel_motor();
    last_wake = xTaskGetTickCount();        //get now system tick to delay a period
    TickType_t previous_sample = last_wake;
    const auto reset_controllers = [&] {
        angle_pid.reset();
        velocity_pid.reset();
        difference_pid.reset();
        roll_pid.reset();
        angle_target = difference_pwm = 0.0F;
        feedback.armed = false;
        feedback.left_pwm = feedback.right_pwm = 0;
    };
    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        const TickType_t sample_tick = xTaskGetTickCount();
        const TickType_t sample_gap = sample_tick - previous_sample;
        previous_sample = sample_tick;
        ++feedback.loop_count;
        feedback.sample_tick = sample_tick;
        feedback.max_sample_gap_ticks = std::max(feedback.max_sample_gap_ticks, sample_gap);
        if (sample_gap > period) { ++feedback.deadline_misses; }
        if (!status_.control_enabled() || control_.storage_blocks_control()) {
            (void)control_.consume_storage_reset();
            startup_gate.reset();
            reset_controllers();
            wheel_motor.forceStop();
            control_.publish_feedback(feedback);
            last_wake = sample_tick;
            continue;
        }
        ControlParameters parameters = control_.parameters();
        if (parameters.motion_command_tick != last_motion_command_tick) {
            last_motion_command_tick = parameters.motion_command_tick;
            feedback.remote_timed_out = false;
        }
        feedback.remote_timed_out = parameters.motion_command_received &&
            (feedback.remote_timed_out ||
             static_cast<TickType_t>(sample_tick - parameters.motion_command_tick) >=
                 task_config::remote_timeout);
        if (feedback.remote_timed_out) {
            // Keep balancing and holding leg height when the controller disappears.
            // Work on the local snapshot; only commands publish parameters.
            parameters.velocity_target = parameters.difference_target = parameters.roll_target = 0.0F;
        }
        feedback.velocity_target = parameters.velocity_target;
        feedback.difference_target = parameters.difference_target;
        feedback.roll_target = parameters.roll_target;
        const float common_height = BalanceCompensation::clampLegHeight(parameters.leg_height);
        // get IMU euler angle
        feedback.imu_valid = imu.getEulerAngleGyro(angle, gyro) &&
            std::isfinite(angle.Roll) && std::isfinite(angle.Pitch) && std::isfinite(angle.Yaw) &&
            std::isfinite(gyro[0]) && std::isfinite(gyro[1]) && std::isfinite(gyro[2]);
        if (!feedback.imu_valid) {
            startup_gate.reset();
            reset_controllers();
            wheel_motor.forceStop();
            if (++imu_read_failures >= 3U) {
                status_.enter_runtime_fault();
                board_.force_safe_outputs();
                board_.command_uart().print("[runtime][FAIL] imu read; control stopped\n");
            }
            control_.publish_feedback(feedback);
            continue;
        }
        imu_read_failures = 0U;
        feedback.euler[0] = static_cast<float>(angle.Roll);
        feedback.euler[1] = static_cast<float>(angle.Pitch);
        feedback.euler[2] = static_cast<float>(angle.Yaw);
        if(parameters.show_imu) {
            board_.command_uart().print("{:07.3f},{:07.3f},{:07.3f}\n", angle.Roll, angle.Pitch, angle.Yaw);
        }
        const float gate_height = feedback.armed
            ? BalanceCompensation::averageLegHeight(legs.left, legs.right) : common_height;
        const float gate_pitch = static_cast<float>(angle.Pitch) +
            BalanceCompensation::pitchBias(parameters.angle_bias, gate_height);
        const float gyro_rate = static_cast<float>(std::sqrt(
            gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]) * 57.295779513);
        if (control_.consume_storage_reset()) {
            startup_gate.reset();
            reset_controllers();
            last_wake = sample_tick;
        }
        // A missed period cannot count as continuous stable startup samples.
        if (sample_gap > period && !feedback.armed) { startup_gate.reset(); }
        feedback.armed = startup_gate.update(feedback.imu_valid, gate_pitch,
            static_cast<float>(angle.Roll), gyro_rate, parameters.velocity_target,
            parameters.difference_target);
        if (!feedback.armed) {
            reset_controllers();
            last_target_roll = parameters.roll_target;
            legs = {common_height, common_height};
            feedback.angle_bias = BalanceCompensation::pitchBias(parameters.angle_bias, common_height);
            feedback.angle_kp = parameters.angle_kp_auto
                ? MotionSettings::effectiveAngleKp(parameters.angle.kp, common_height) : parameters.angle.kp;
            feedback.pitch_error = static_cast<float>(angle.Pitch) + feedback.angle_bias;
            wheel_motor.forceStop();
            if (++velocity_loop_count >= 5U) {
                velocity_loop_count = 0U;
                (void)left_encoder.getRPM();
                (void)right_encoder.getRPM();
                control_.publish_leg_targets(legs);
                (void)servo_.notify_give();
            }
            control_.publish_feedback(feedback);
            continue;
        }
        velocity_loop_count++;
        if(velocity_loop_count >= 5){
            velocity_loop_count = 0;
            double left_rpm = left_encoder.getRPM();
            double right_rpm = right_encoder.getRPM();
            double average_rpm = (left_rpm+right_rpm)/2;
            difference_rpm = left_rpm - right_rpm;
            velocity_pid.setTunings(parameters.velocity.kp,parameters.velocity.ki,parameters.velocity.kd);
            difference_pid.setTunings(parameters.difference.kp,parameters.difference.ki,parameters.difference.kd);
            angle_target = velocity_pid.update(parameters.velocity_target,average_rpm);
            difference_pwm = difference_pid.update(parameters.difference_target,difference_rpm);
            if(parameters.show_rpm){
                board_.command_uart().print("A: {:07.3f}\tB: {:07.3f}\n",left_rpm,right_rpm);
            }
            //roll pid
            roll_pid.setTunings(parameters.roll.kp,parameters.roll.ki,parameters.roll.kd);
            float roll_error = parameters.roll_target - angle.Roll;
            // 检测目标角度是否跨越零点（正负号改变）
            if ((last_target_roll > 0 && parameters.roll_target < 0) || (last_target_roll < 0 && parameters.roll_target > 0)) {
                roll_pid.reset(); // 清除旧的增量累加值 last_out_ 和积分项
            }
            last_target_roll = parameters.roll_target;
            float adjust_y = roll_pid.updateIncremental(parameters.roll_target,angle.Roll);
            float geometric_comp_y;
            const float threshold_degrees = 3.0f;      // 触发补偿的 Roll 角阈值 (度)
            const float compensation_gain = 0.5f;             // 补偿系数 (0.0~1.0)，建议先给 0.8，避免过冲
            // 使用平滑死区处理误差，避免补偿量突变导致舵机抽搐
            if (roll_error > 3.0f) {
                // 仅对超出阈值的部分进行正弦补偿
                geometric_comp_y = compensation_gain * 55.0 * std::sin((roll_error - threshold_degrees) * 0.0174532925f);
                adjust_y += geometric_comp_y;
            }
            else if (roll_error < -3.0f) {
                geometric_comp_y = compensation_gain * 55.0 * std::sin((roll_error + threshold_degrees) * 0.0174532925f);
                adjust_y += geometric_comp_y;
            }
            legs.left = common_height - adjust_y;
            legs.right = common_height + adjust_y;
            // Clamp before publishing; motion is the sole writer of leg targets.
            legs.left = std::clamp(legs.left, 44.5F, 78.5F);
            legs.right = std::clamp(legs.right, 44.5F, 78.5F);
            control_.publish_leg_targets(legs);
            (void)servo_.notify_give();
        }
        const float average_height = BalanceCompensation::averageLegHeight(legs.left, legs.right);
        feedback.angle_bias = BalanceCompensation::pitchBias(parameters.angle_bias, average_height);
        feedback.angle_kp = parameters.angle_kp_auto
            ? MotionSettings::effectiveAngleKp(parameters.angle.kp, average_height) : parameters.angle.kp;
        angle_pid.setTunings(feedback.angle_kp,parameters.angle.ki,parameters.angle.kd);
        feedback.pitch_error = static_cast<float>(angle.Pitch) + feedback.angle_bias;
        float even_pwm = angle_pid.update(angle_target,angle.Pitch + feedback.angle_bias);
        int left_pwm = static_cast<int>(std::round((even_pwm + difference_pwm)));
        int right_pwm = static_cast<int>(std::round((even_pwm - difference_pwm)));
        left_pwm = TB6612::clamp(left_pwm,1000,-1000);
        right_pwm = TB6612::clamp(right_pwm,1000,-1000);
        feedback.left_pwm = left_pwm;
        feedback.right_pwm = right_pwm;
        // Only the bounded PWM register writes and feedback publication are
        // atomic with begin_storage/control off; sensor I/O and PID math stay outside.
        taskENTER_CRITICAL();
        const bool storage_reset = control_.consume_storage_reset();
        if (!status_.control_enabled() || control_.storage_blocks_control() || storage_reset) {
            startup_gate.reset();
            reset_controllers();
            wheel_motor.forceStop();
            last_wake = xTaskGetTickCount();
        } else {
            wheel_motor.setA_DeadZone(parameters.motor_deadzone);
            wheel_motor.setB_DeadZone(parameters.motor_deadzone);
            wheel_motor.setAVel_raw(left_pwm);
            wheel_motor.setBVel_raw(right_pwm);
        }
        control_.publish_feedback(feedback);
        taskEXIT_CRITICAL();
    }
}

} // namespace app
