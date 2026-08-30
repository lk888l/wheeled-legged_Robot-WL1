/********************************************************************************
  * @file           : MPU6050.h
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/


#ifndef __MPU6050_H
#define __MPU6050_H


#include <algorithm>
#include <cmath>
#include <cstdint>

#ifndef MPU6050_HOST_TEST
#include "main.h"
#else
struct I2C_HandleTypeDef
{
};

enum HAL_StatusTypeDef
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
};

extern "C" HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef* hi2c,
                                                std::uint16_t device_address,
                                                std::uint16_t memory_address,
                                                std::uint16_t memory_address_size,
                                                std::uint8_t* data,
                                                std::uint16_t size,
                                                std::uint32_t timeout);
extern "C" HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef* hi2c,
                                                 std::uint16_t device_address,
                                                 std::uint16_t memory_address,
                                                 std::uint16_t memory_address_size,
                                                 std::uint8_t* data,
                                                 std::uint16_t size,
                                                 std::uint32_t timeout);
#endif

#include "vqf.hpp"


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
    static constexpr uint32_t MPU6050_TIME_OUT = 2;
    static constexpr std::uint16_t FUSION_SAMPLE_RATE_HZ = 100;
    static constexpr std::uint8_t DLPF_CONFIG = 0x03;
    static constexpr std::int32_t RAW_SATURATION_THRESHOLD = 31129;
    static constexpr double MIN_TRUSTED_ACCEL_G = 0.8;
    static constexpr double MAX_TRUSTED_ACCEL_G = 1.2;
    // Magnitude-only gating accepts a horizontal 0.5 g acceleration because
    // sqrt(1^2 + 0.5^2) remains inside the window. Compare its direction with
    // gyro-predicted gravity as well, with hysteresis to avoid chatter.
    static constexpr double ACCEL_DIRECTION_REJECT_COS = 0.9781476007338057; // cos 12 deg
    static constexpr double ACCEL_DIRECTION_ACCEPT_COS = 0.9945218953682733; // cos 6 deg
    static constexpr double ACCEL_CANDIDATE_STEP_COS = 0.9996573249755573; // cos 1.5 deg
    static constexpr double ACCEL_CANDIDATE_MEAN_COS = 0.9986295347545738; // cos 3 deg
    static constexpr double MIN_INITIAL_ACCEL_G = 0.90;
    static constexpr double MAX_INITIAL_ACCEL_G = 1.10;
    static constexpr double ACCEL_REFERENCE_TOLERANCE_G = 0.03;
    static constexpr double ACCEL_CANDIDATE_NORM_STEP_G = 0.02;
    // The legacy board calibration has about 2.75 deg/s of combined static
    // offset. Leave enough margin for temperature drift while still requiring
    // a stable gravity direction and an explicit external stationary hint.
    static constexpr double MAX_REACQUIRE_GYRO_RAD_PER_SECOND = 0.0872664625997165;
    static constexpr std::uint16_t ACCEL_REACQUIRE_SAMPLES = 30;
    static constexpr double VQF_ACCEL_TIME_CONSTANT_SECONDS = 0.5;
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
        GyroRange_t GyroRange = GyroRange_t::G2000;
        AccRange_t AccRange = AccRange_t::A8;
        uint16_t SampleRate = FUSION_SAMPLE_RATE_HZ;       //(hz)
        double GyroOffset[3]{};
    }InitConfig_t;

    typedef struct EulerAngle {
        double Roll{};
        double Pitch{};
        double Yaw{};
    }EulerAngle;

    enum SaturationFlag : std::uint8_t
    {
        AccelXRawSaturated = 1U << 0U,
        AccelYRawSaturated = 1U << 1U,
        AccelZRawSaturated = 1U << 2U,
        GyroXRawSaturated = 1U << 3U,
        GyroYRawSaturated = 1U << 4U,
        GyroZRawSaturated = 1U << 5U,
    };

    /**
     * One coherent MPU6050 sample. Acceleration is expressed in m/s^2 and
     * angular velocity in rad/s, matching the VQF input units.
     *
     * valid means that the I2C sample produced a finite attitude, the filter
     * has seen at least one gravity-trusted accelerometer sample, and the
     * gyroscope was not close to its raw full-scale limit. A rejected dynamic
     * acceleration does not invalidate gyro-only attitude propagation; use
     * accel_trusted and saturation_flags for diagnostics.
     */
    struct Sample
    {
        EulerAngle angle{};
        double gyro[3]{};
        double accel[3]{};
        std::int16_t raw_gyro[3]{};
        std::int16_t raw_accel[3]{};
        float temperature_c{};
        std::uint8_t saturation_flags{};
        bool accel_trusted{};
        bool valid{};
    };

private:
    I2C_HandleTypeDef* Hi2c = nullptr;
    InitConfig_t M650_cfg;
    VQF vqf;
    double GyroCoefficient{};
    double AccCoefficient{};
    bool has_trusted_accel_{};
    bool accel_direction_trusted_ = false;
    std::uint16_t accel_reacquire_samples_ = 0;
    bool has_accel_norm_reference_ = false;
    double accel_norm_reference_g_ = 1.0;
    double accel_candidate_direction_sum_[3]{};
    double previous_accel_candidate_direction_[3]{};
    double accel_candidate_norm_sum_g_ = 0.0;

    struct RawSample
    {
        std::int16_t accel[3]{};
        std::int16_t temperature{};
        std::int16_t gyro[3]{};
    };

    bool readRawSample(RawSample& sample);
    void convertRawGyro(const std::int16_t raw[3], double gyro_degrees_per_second[3]) const;
    void convertRawAccel(const std::int16_t raw[3], double accel_g[3]) const;
    static std::uint8_t getSaturationFlags(const std::int16_t raw_accel[3],
                                           const std::int16_t raw_gyro[3]);
    void resetAccelCandidate();
    bool tryStableAccelReacquisition(const double accel_g[3],
                                     double accel_norm_g,
                                     double gyro_norm_rad_per_second,
                                     bool stationary_hint);

public:
    MPU6050(I2C_HandleTypeDef* _hi2c);
    MPU6050(I2C_HandleTypeDef* _hi2c,InitConfig_t _cfg);
    bool Init();
    bool getGyro(double _gyro[3]);
    bool getAccel(double _acc[3]);
    bool getTemperature(float& _temp);
    bool getEulerAngle(EulerAngle& _angle);
    bool getEulerAngle(EulerAngle& _angle, bool stationary_hint);
    bool getEulerAngleACC(EulerAngle& _angle, double _acc[3]);
    bool getEulerAngleACC(EulerAngle& _angle,
                          double _acc[3],
                          bool stationary_hint);
    /** Legacy interface: returns angular velocity in deg/s. */
    bool getEulerAngleGyro(EulerAngle& _angle, double _gyro[3]);
    bool getEulerAngleGyro(EulerAngle& _angle,
                           double _gyro[3],
                           bool stationary_hint);

    /**
     * Returns true when the coherent 14-byte I2C transfer succeeded. Inspect
     * sample.valid separately to decide whether the fused attitude is usable.
     */
    bool getSample(Sample& sample, bool stationary_hint = false);

    /** Discard fusion history after a missed/saturated gyro interval. */
    void resetFusion();

    /**
     * Converts and fuses an already coherent raw sensor sample. This is useful
     * for host tests and provides a future migration path to DATA_RDY/FIFO DMA.
     * The return value is identical to sample.valid; no I2C transfer occurs.
     */
    bool processRawSample(const std::int16_t raw_accel[3],
                          const std::int16_t raw_gyro[3],
                          Sample& sample,
                          std::int16_t raw_temperature = 0,
                          bool stationary_hint = false);
    void setGyroOffset(double&& _xg, double&& _yg, double&& _zg);
    void setGyroOffset(double _offnum[3]);

    static double gyroCoefficientForRange(GyroRange_t range);
    static double accelCoefficientForRange(AccRange_t range);

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
        _angle.Roll = std::atan2(2 * (_q[0] * _q[1] + _q[2] * _q[3]),
                                 _q[0]*_q[0] - _q[1]*_q[1] - _q[2]*_q[2] + _q[3]*_q[3])
            * EULERANGLE_COE;
        const double pitch_sine = std::clamp(
            2 * (_q[1] * _q[3] - _q[0] * _q[2]), -1.0, 1.0);
        _angle.Pitch = -std::asin(pitch_sine) * EULERANGLE_COE;
        _angle.Yaw = std::atan2(2 * (_q[0] * _q[3] + _q[1] * _q[2]),
                                _q[0]*_q[0] + _q[1]*_q[1] - _q[2]*_q[2] - _q[3]*_q[3])
            * EULERANGLE_COE;
    }
};


#endif //__MPU6050_H
