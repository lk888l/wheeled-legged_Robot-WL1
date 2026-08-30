/********************************************************************************
  * @file           : PID.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-4-5
  *******************************************************************************/

#ifdef __GNUC__
#pragma once
#endif //__GNUC__
#ifndef __F411CEU6_PID_HPP
#define __F411CEU6_PID_HPP

#include <cmath>


class PID {
public:
    /**
    * @brief PID 构造函数
    * @param kp 比例系数
    * @param ki 积分系数
    * @param kd 微分系数
    * @param min_out 输出下限
    * @param max_out 输出上限
    */
    PID(float kp, float ki, float kd, float min_out, float max_out, float min_int, float max_int)
            : kp_(kp), ki_(ki), kd_(kd), min_out_(min_out), max_out_(max_out), min_int_(min_int),max_int_(max_int),
              integral_(0.0f), prev_error_(0.0f), prevTWO_error_(0.0f), prev_actual(0.0f) {}

    /**
    * @brief 计算 PID 输出
    * @param target
    * @param measured
    * @param dt 采样周期 (ms)
    * @return
    */
    float update(float target, float measured) {
        return updateWithTimeRatio(target, measured, 1.0F);
    }

    /**
     * Update with a measured interval while retaining gains tuned for the
     * nominal interval. Invalid intervals deliberately fall back to the legacy
     * one-sample behavior.
     */
    float update(float target,
                 float measured,
                 float actual_dt_seconds,
                 float nominal_dt_seconds) {
        return updateWithTimeRatio(
            target, measured, normalizedTimeRatio(actual_dt_seconds, nominal_dt_seconds));
    }

    /**
     * @brief 增量式 PID 计算
     * \n     增量公式：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
     * @param target
     * @param measured
     * @return
     */
    float updateIncremental(float target, float measured){
        return updateIncrementalWithTimeRatio(target, measured, 1.0F);
    }

    /** Variable-interval form of updateIncremental(). */
    float updateIncremental(float target,
                            float measured,
                            float actual_dt_seconds,
                            float nominal_dt_seconds) {
        return updateIncrementalWithTimeRatio(
            target, measured, normalizedTimeRatio(actual_dt_seconds, nominal_dt_seconds));
    }

    /**
    * @brief
    */
    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        prevTWO_error_ = 0.0f;
        last_out_ = 0.0f;
        prev_actual = 0.0f;
        previous_time_ratio_ = 1.0F;
    }

    /**
     * @brief Prime the history for a bumpless first update after a mode change.
     * This preserves the integrator/output while preventing a derivative kick.
     */
    void prime(float target, float measured) {
        prev_error_ = target - measured;
        prevTWO_error_ = prev_error_;
        prev_actual = measured;
        previous_time_ratio_ = 1.0F;
    }

    /** Reset all accumulated state and prime it at the current operating point. */
    void reset(float target, float measured) {
        reset();
        prime(target, measured);
    }

    /**
     * @brief 动态调参接口
     * @param kp
     * @param ki
     * @param kd
     */
    void setTunings(float kp, float ki, float kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    void setLimits(float min_out, float max_out, float min_int, float max_int) {
        if (min_out > max_out || min_int > max_int) {
            return;
        }
        min_out_ = min_out;
        max_out_ = max_out;
        min_int_ = min_int;
        max_int_ = max_int;
        if (integral_ > max_int_) integral_ = max_int_;
        else if (integral_ < min_int_) integral_ = min_int_;
        if (last_out_ > max_out_) last_out_ = max_out_;
        else if (last_out_ < min_out_) last_out_ = min_out_;
    }

private:
    static float normalizedTimeRatio(float actual_dt_seconds,
                                     float nominal_dt_seconds) {
        if (!std::isfinite(actual_dt_seconds) || actual_dt_seconds <= 0.0F
            || !std::isfinite(nominal_dt_seconds) || nominal_dt_seconds <= 0.0F) {
            return 1.0F;
        }
        const float ratio = actual_dt_seconds / nominal_dt_seconds;
        return std::isfinite(ratio) && ratio > 0.0F ? ratio : 1.0F;
    }

    float updateWithTimeRatio(float target, float measured, float time_ratio) {
        // 1. 计算误差
        float error = target - measured;

        // 2. 积分项 (包含积分限幅防止饱和)
        if(ki_!=0){
            integral_ += time_ratio == 1.0F ? error : error * time_ratio;
            // 积分抗饱和 (Simple Clamping)
            if (integral_ > max_int_) { integral_ = max_int_; }
            else if (integral_ < min_int_) { integral_ = min_int_; }
        }
        else{
            integral_ = 0;
        }

        // 3. 总输出并再次限幅
        const float measured_delta = measured - prev_actual;
        const float normalized_measured_delta = time_ratio == 1.0F
            ? measured_delta
            : measured_delta / time_ratio;
        float total_out = kp_ * error
                        + integral_ * ki_
//                        + kd_ * (error - prev_error_);
                        - kd_ * normalized_measured_delta;   //微分先行计算公式

        if (total_out > max_out_) total_out = max_out_;
        else if (total_out < min_out_) total_out = min_out_;

        // 保存状态
        prev_error_ = error;
        prev_actual = measured;
        previous_time_ratio_ = time_ratio;
        return total_out;
    }

    float updateIncrementalWithTimeRatio(float target,
                                         float measured,
                                         float time_ratio) {
        if(kp_==0 && ki_==0)    {return 0.0f;}
        float error = target - measured;
        // 计算增量 Δu
        float delta_out = 0.0F;
        if (time_ratio == 1.0F && previous_time_ratio_ == 1.0F) {
            // Preserve the exact legacy arithmetic at the nominal interval.
            delta_out = kp_ * (error - prev_error_)
                        + ki_ * error
                        + kd_ * (error - 2.0f * prev_error_ + prevTWO_error_);
        }
        else {
            const float current_error_delta = error - prev_error_;
            const float previous_error_delta = prev_error_ - prevTWO_error_;
            delta_out = kp_ * current_error_delta
                        + ki_ * error * time_ratio
                        + kd_ * ((current_error_delta / time_ratio)
                                 - (previous_error_delta / previous_time_ratio_));
        }
        // 累加到当前输出
        last_out_ += delta_out;

        // 输出限幅
        if (last_out_ > max_out_) last_out_ = max_out_;
        else if (last_out_ < min_out_) last_out_ = min_out_;
        // 更新历史误差
        prevTWO_error_ = prev_error_;
        prev_error_ = error;
        prev_actual = measured;
        previous_time_ratio_ = time_ratio;
        return last_out_;
    }
    float kp_, ki_{}, kd_;
    float min_out_, max_out_;
    float min_int_, max_int_;
    float integral_;
    float prev_error_;
    float prevTWO_error_;
    float prev_actual;
    float last_out_{};
    float previous_time_ratio_ = 1.0F;
};


#endif //__F411CEU6_PID_HPP
