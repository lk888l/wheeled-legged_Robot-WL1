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

#include <algorithm>

class PID {
public:
    struct Terms { float p{}, i{}, d{}; };
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
              integral_(0.0f), prev_error_(0.0f), prevTWO_error_(0.0f),
              anti_windup_min_(min_out), anti_windup_max_(max_out) {}

    /**
    * @brief 计算 PID 输出
    * @param target
    * @param measured
    * Gains use per-sample integral and measurement difference, not SI time units.
    * @return
    */
    float update(float target, float measured, bool anti_windup = false, bool integrate = true) {
        return updateWithMeasurementDelta(target, measured,
            has_previous_measurement_ ? measured - prev_actual : 0.0F, anti_windup, integrate);
    }

    // External derivative in measurement units per nominal sample. Passing a
    // sensor rate * nominal_period retains the existing discrete Kd scale.
    float updateWithMeasurementDelta(float target, float measured, float measurement_delta,
                                    bool anti_windup = false, bool integrate = true) {
        const float error = target - measured;
        if (ki_ == 0.0F) integral_ = 0.0F;
        float candidate = integral_;
        if (ki_ != 0.0F && integrate) {
            candidate = std::clamp(integral_ + error, min_int_, max_int_);
        }
        terms_.p = kp_ * error;
        terms_.d = -kd_ * measurement_delta;
        const float proposed = terms_.p + ki_ * candidate + terms_.d;
        const float integral_change = ki_ * (candidate - integral_);
        // Reject further windup, but allow unwinding, including negative Ki.
        if (!anti_windup || !((proposed > anti_windup_max_ && integral_change > 0.0F) ||
                             (proposed < anti_windup_min_ && integral_change < 0.0F))) {
            integral_ = candidate;
        }
        terms_.i = ki_ * integral_;
        prev_error_ = error;
        prev_actual = measured;
        has_previous_measurement_ = true;
        return std::clamp(terms_.p + terms_.i + terms_.d, min_out_, max_out_);
    }

    const Terms& terms() const { return terms_; }
    // Account for downstream mixing without reducing proportional/damping authority.
    void setAntiWindupOutputLimits(float minimum, float maximum) {
        anti_windup_min_ = minimum; anti_windup_max_ = maximum;
    }

    /**
     * @brief 增量式 PID 计算
     * \n     增量公式：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
     * @param target
     * @param measured
     * @return
     */
    float updateIncremental(float target, float measured){
        if(kp_==0 && ki_==0 && kd_==0) { reset(); return 0.0f; }
        float error = target - measured;
        // 计算增量 Δu
        float delta_out = kp_ * (error - prev_error_)
                          + ki_ * error
                          + kd_ * (error - 2.0f * prev_error_ + prevTWO_error_);
        // 累加到当前输出
        last_out_ += delta_out;

        // 输出限幅
        if (last_out_ > max_out_) last_out_ = max_out_;
        else if (last_out_ < min_out_) last_out_ = min_out_;
        // 更新历史误差
        prevTWO_error_ = prev_error_;
        prev_error_ = error;
        prev_actual = measured;
        has_previous_measurement_ = true;
        return last_out_;
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
        has_previous_measurement_ = false;
        terms_ = {};
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

private:
    float kp_, ki_{}, kd_;
    float min_out_, max_out_;
    float min_int_, max_int_;
    float integral_;
    float prev_error_;
    float prevTWO_error_;
    float prev_actual{}; // Deterministic first derivative sample; never read stack garbage.
    float last_out_{};
    bool has_previous_measurement_{};
    Terms terms_{};
    float anti_windup_min_, anti_windup_max_;
};


#endif //__F411CEU6_PID_HPP
