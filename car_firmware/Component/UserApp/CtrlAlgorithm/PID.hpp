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
              integral_(0.0f), prev_error_(0.0f), prevTWO_error_(0.0f) {}

    /**
    * @brief 计算 PID 输出
    * @param target
    * @param measured
    * @param sample_ratio actual / nominal period; existing gains are per sample
    * @return
    */
    float update(float target, float measured, float sample_ratio = 1.0F,
                 bool integrate = true) {
        if (!std::isfinite(sample_ratio) || sample_ratio <= 0.0F) return 0.0F;
        const float delta = has_previous_measurement_
            ? (measured - prev_actual) / sample_ratio : 0.0F;
        return updateWithMeasurementRate(target, measured, delta, sample_ratio, integrate);
    }

    // measured_rate is change per NOMINAL sample, not degrees/second.
    // It may come from a gyro, independent of target/balance-bias changes.
    float updateWithMeasurementRate(float target, float measured, float measured_rate,
                                    float sample_ratio = 1.0F, bool integrate = true) {
        if (!std::isfinite(sample_ratio) || sample_ratio <= 0.0F) return 0.0F;
        const float error = target - measured;
        const float pd = kp_ * error - kd_ * measured_rate;
        if (ki_ == 0.0F) {
            integral_ = 0.0F;
        } else if (integrate) {
            float candidate = std::clamp(integral_ + error * sample_ratio,
                                          min_int_, max_int_);
            const float candidate_output = pd + ki_ * candidate;
            const float integral_push = ki_ * (candidate - integral_);
            // Permit unwinding, including negative Ki; reject accumulation
            // beyond saturation. Keep the partial step up to the limit so a
            // pure I controller can still reach it with a large error sample.
            if (candidate_output > max_out_ && integral_push > 0.0F) {
                candidate = pd + ki_ * integral_ >= max_out_
                    ? integral_ : (max_out_ - pd) / ki_;
            } else if (candidate_output < min_out_ && integral_push < 0.0F) {
                candidate = pd + ki_ * integral_ <= min_out_
                    ? integral_ : (min_out_ - pd) / ki_;
            }
            integral_ = std::clamp(candidate, min_int_, max_int_);
        }
        prev_error_ = error;
        prev_actual = measured;
        has_previous_measurement_ = true;
        return std::clamp(pd + ki_ * integral_, min_out_, max_out_);
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
};


#endif //__F411CEU6_PID_HPP
