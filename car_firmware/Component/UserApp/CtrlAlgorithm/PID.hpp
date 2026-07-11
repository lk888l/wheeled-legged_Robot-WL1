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
    * @param dt 采样周期 (ms)
    * @return
    */
    float update(float target, float measured) {
        // 1. 计算误差
        float error = target - measured;

        // 2. 积分项 (包含积分限幅防止饱和)
        if(ki_!=0){
            integral_ += error;
            // 积分抗饱和 (Simple Clamping)
            if (integral_ > max_int_) { integral_ = max_int_; }
            else if (integral_ < min_int_) { integral_ = min_int_; }
        }
        else{
            integral_ = 0;
        }

        // 3. 总输出并再次限幅
        float total_out = kp_ * error
                        + integral_ * ki_
//                        + kd_ * (error - prev_error_);
                        - kd_ * (measured - prev_actual);   //微分先行计算公式

        if (total_out > max_out_) total_out = max_out_;
        else if (total_out < min_out_) total_out = min_out_;

        // 保存状态
        prev_error_ = error;
        prev_actual = measured;
        return total_out;
    }

    /**
     * @brief 增量式 PID 计算
     * \n     增量公式：Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
     * @param target
     * @param measured
     * @return
     */
    float updateIncremental(float target, float measured){
        if(kp_==0 && ki_==0)    {return 0.0f;}
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
    float prev_actual;
    float last_out_{};
};


#endif //__F411CEU6_PID_HPP
