/********************************************************************************
  * @file           : MPU6050.h
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/


#ifndef __MPU6050_H
#define __MPU6050_H


#include "main.h"
#include "vqf.hpp"
#include <memory>


class MPU6050 {
    static constexpr uint8_t WHO_AM_I_REG   = 0x75;
    static constexpr uint8_t PWR_MGMT_1_REG = 0x6B;
    static constexpr uint8_t PWR_MGMT_2_REG = 0x6C;
    static constexpr uint8_t SMPLRT_DIV_REG = 0x19;
    static constexpr uint8_t MPU_CFG_REG    = 0X1A;
    static constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
    static constexpr uint8_t ACCEL_XOUT_H_REG = 0x3B;
    static constexpr uint8_t MPU_INTBP_CFG_REG = 0X37;
    static constexpr uint8_t TEMP_OUT_H_REG = 0x41;
    static constexpr uint8_t GYRO_CONFIG_REG = 0x1B;
    static constexpr uint8_t GYRO_XOUT_H_REG = 0x43;
    static constexpr uint8_t MPU6050_ADDR = 0xD0;
    static constexpr uint8_t MPU6050_TIME_OUT = 100;
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD_COE = (PI / 180.0f);
    static constexpr double EULERANGLE_COE = 57.295779513082320876798154814105;

public:
    enum class GyroRange_t : uint8_t {
        G250 = 0x00,
        G500,
        G1000,
        G2000
    };
    enum class AccRange_t : uint8_t {
        A2 = 0x00,
        A4,
        A8,
        A16
    };
    typedef struct InitConfig_t {
        GyroRange_t GyroRange = GyroRange_t::G1000;
        AccRange_t AccRange = AccRange_t::A4;
        uint16_t SampleRate = 100;       //(hz)
        double GyroOffset[3]{};
    }InitConfig_t;

    typedef struct EulerAngle {
        double Roll;
        double Pitch;
        double Yaw;
    }EulerAngle;

private:
    I2C_HandleTypeDef* Hi2c = nullptr;
    InitConfig_t M650_cfg;
    VQF vqf;
    double GyroCoefficient{};
    double AccCoefficient{};

public:
    MPU6050(I2C_HandleTypeDef* _hi2c);
    MPU6050(I2C_HandleTypeDef* _hi2c,InitConfig_t _cfg);
    bool Init();
    bool getGyro(double _gyro[3]);
    bool getAccel(double _acc[3]);
    bool getTemperature(float& _temp);
    bool getEulerAngle(EulerAngle& _angle);
    bool getEulerAngleACC(EulerAngle& _angle, double _acc[3]);
    bool getEulerAngleGyro(EulerAngle& _angle, double _gyro[3]);
    void setGyroOffset(double&& _xg, double&& _yg, double&& _zg);
    void setGyroOffset(double _offnum[3]);

    /**
 * @brief
 * @param frequency_hz
 * @return
 */
    static inline double HZ_toms(double frequency_hz) {
        // 防除零错误：频率不能为0或负数
//        if (frequency_hz <= 0) {
//            return 0;
//        }
        return 1000.0 / frequency_hz;
    }

    /**
     * @brief
     * @param period_ms
     * @return
     */
    static inline double ms_toHZ(double period_ms) {
        // 防除零错误：周期不能为0或负数（无意义）
//        if (period_ms <= 0) {
//            return 0;
//        }
        return 1000.0 / period_ms;
    }

    static inline double DegTorad(double _rad)
    {
        return _rad * DEG_TO_RAD_COE;
    }

    static inline void DegTorad(double rad[3]){
        rad[0] *= DEG_TO_RAD_COE;
        rad[1] *= DEG_TO_RAD_COE;
        rad[2] *= DEG_TO_RAD_COE;
    }

    static inline void GToMS2(double acc[3]){
        acc[0] *= 9.81;
        acc[1] *= 9.81;
        acc[2] *= 9.81;
    }

    static inline void QuatToEuler(const double _q[4], EulerAngle& _angle)
    {
        _angle.Roll = atan2(2 * (_q[0] * _q[1] + _q[2] * _q[3]), _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3])* EULERANGLE_COE;
        _angle.Pitch = -asin(2 * (_q[1] * _q[3] - _q[0] * _q[2]))*EULERANGLE_COE;
        _angle.Yaw = atan2(2 * (_q[0] * _q[3] + _q[1] * _q[2]), _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3])*EULERANGLE_COE;
    }
};


#endif //__MPU6050_H
