/********************************************************************************
  * @file           : LegKinematics.hpp
  * @author         : Luka
  * @brief          : None
  * @attention      : 四连杆单自由度运动学正逆解
  * @date           : 26-4-15
  *******************************************************************************/


#include <cmath>
#include <cstdint>

#ifndef __LEGKINEMATICS_HPP
#define ___LEGKINEMATICS_HPP


class LegKinematics {
    struct Point{
        float x;
        float y;
    };
public:
    static constexpr float L_CD = 40.0;  // 舵机驱动的红杆
    static constexpr float L_AB = 40.0;  // 上连杆
    static constexpr float L_BD = 20.0;  // 右连杆
    static constexpr float L_DW = 50.0;  // D点到轮心W的腿长

// 固接点A的坐标 (以舵机C为原点，竖直向下为Y正方向，向右为X正方向)
    static constexpr float A_X = 15.0;
    static constexpr float A_Y = -20.0;

/**
 * @brief 正运动学：输入电机角度，计算并返回轮端所在的高度(Y坐标)
 * @param theta_deg 电机驱动杆(红杆)与水平方向的夹角 (度)
 * @return float 轮心W的Y坐标。如果机构无解(死点)，返回 -1.0
 */
    static Point forwardKinematics(float theta_deg) {
        // 将角度转换为弧度
        float theta_rad = theta_deg * M_PI / 180.0;

        // 1. 计算红杆末端 D 点坐标
        float D_x = L_CD * cos(theta_rad);
        float D_y = L_CD * sin(theta_rad);

        // 2. 求 B 点坐标 (求解以A为圆心L_AB为半径，和以D为圆心L_BD为半径的两个圆的交点)
        float dx = D_x - A_X;
        float dy = D_y - A_Y;
        float d2 = dx * dx + dy * dy;
        float d = sqrt(d2);

        // 奇异点/无解保护：若两圆不相交
        if (d > (L_AB + L_BD) || d < fabs(L_AB - L_BD)) {
            return {-1.0f,-1.0f};
        }

        float a = (L_AB * L_AB - L_BD * L_BD + d2) / (2 * d);
        float h2 = L_AB * L_AB - a * a;
        float h = (h2 > 0) ? sqrt(h2) : 0;

        float P2_x = A_X + (a / d) * dx;
        float P2_y = A_Y + (a / d) * dy;

        // 根据你的图纸装配模式，B点应位于D点的右侧/上方，取正解
        float B_x = P2_x + (h / d) * dy;
        float B_y = P2_y - (h / d) * dx;

        // 3. 计算轮心 W 的坐标 (假设 B, D, W 为同一刚体且共线)
        // 向量关系: DW = (50/20) * BD = 2.5 * BD
        // W_y = D_y + 2.5 * (D_y - B_y)
        float W_y = D_y + (L_DW / L_BD) * (D_y - B_y);  //
        float W_x = D_x + (L_DW / L_BD) * (D_x - B_x);  //
        return {W_x,W_y};
    }

/**
 * @brief 逆运动学：输入目标高度，返回所需的电机角度 (二分查找法)
 * @param target_Y 目标轮端高度 (mm)
 * @return float 目标电机角度 (度)
 */
    static float getMotorAngleForHeight(float target_Y, float* result_x = nullptr) {
        float min_angle = 0.0;   // 舵机最小工作角度 (可根据实际情况调整)
        float max_angle = 80.0;  // 舵机最大工作角度
        float mid_angle = 0.0;

        Point point;
        // 迭代 15 次，精度可达 80/(2^15) ≈ 0.002度，满足所有电机控制需求
        for (int i = 0; i < 15; i++) {
            mid_angle = (min_angle + max_angle) / 2.0;
            point = forwardKinematics(mid_angle);

            if (point.y < 0) {
                // 遇到死点，缩小范围避开奇异区
                max_angle = mid_angle;
                continue;
            }

            // 因为高度Y随角度增加而单调增加
            if (point.y < target_Y) {
                min_angle = mid_angle; // 当前高度不够，需要更大的角度
            } else {
                max_angle = mid_angle; // 当前高度过高，需要更小的角度
            }
        }

        if(result_x != nullptr) {*result_x = point.x;}
        // 返回前可以加上你的系统标定误差 offset
        // 例如：return mid_angle + servo_offset;
        return mid_angle;
    }
};


#endif //__LEGKINEMATICS_HPP