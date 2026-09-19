/********************************************************************************
  * @file           : MPU6050.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/


#include <utility>
#include <algorithm>
#include <cmath>
#include "MPU6050.h"

namespace {
VQFParams fusionParameters()
{
    VQFParams params;
    // This MPU6050 has ~3.4 deg/s Z bias at rest. VQF's default 2 deg/s
    // absolute bias limit prevents rest detection, so it cannot calibrate it.
    // Keep the normal rest variance/time tests; allow a bounded 5 deg/s bias.
    params.biasClip = 5.0;
    params.biasSigmaInit = 2.0;
    return params;
}
}

MPU6050::MPU6050(I2C_HandleTypeDef *_hi2c)
    : Hi2c(_hi2c)
    ,vqf(fusionParameters(), samplePeriod(M650_cfg.SampleRate))
{

}

MPU6050::MPU6050(I2C_HandleTypeDef *_hi2c, MPU6050::InitConfig_t _cfg)
    : Hi2c(_hi2c)
    , M650_cfg(_cfg)
    ,vqf(fusionParameters(), samplePeriod(M650_cfg.SampleRate))
{

}

// Period between host fusion updates, not between the sensor's register updates.
// Keep the existing integer-millisecond cadence, including invalid/zero rates.
double MPU6050::samplePeriod(uint16_t requested)
{
    const auto rate = std::clamp<uint16_t>(requested, 4U, 1000U);
    return static_cast<double>(1000U / rate) / 1000.0;
}

bool MPU6050::Init()
{
    if (Hi2c == nullptr || static_cast<uint8_t>(M650_cfg.AccRange) > 3U ||
        static_cast<uint8_t>(M650_cfg.GyroRange) > 3U) { return false; }
    const auto read = [&](uint8_t reg, uint8_t& value) {
        return HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, reg, 1, &value, 1,
                               MPU6050_TIME_OUT) == HAL_OK;
    };
    const auto write = [&](uint8_t reg, uint8_t value) {
        return HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, reg, 1, &value, 1,
                                MPU6050_TIME_OUT) == HAL_OK;
    };
    uint8_t identity{};
    if (!read(WHO_AM_I_REG, identity) || identity != 0x68) { return false; }
    // Reset stale state after MCU-only resets, then retain the gyro X PLL clock.
    if (!write(PWR_MGMT_1_REG, 0x80)) { return false; }
    HAL_Delay(100U);
    if (!write(PWR_MGMT_1_REG, 0x01) || !write(PWR_MGMT_2_REG, 0x00)) { return false; }
    HAL_Delay(30U);
    const uint16_t divider = 1000U / std::clamp<uint16_t>(M650_cfg.SampleRate, 4U, 1000U);
    // Preserve the measurement timing used to tune the balance controller before
    // ba5b8e7. DLPF=0 uses the 8 kHz gyro base clock: at a 100 Hz host cadence,
    // divider=10 gives 800 Hz register updates. VQF still advances by 10 ms.
    // Selecting DLPF=3 from the host rate changed both the filter delay and the
    // register rate (to 100 Hz), adding feedback lag without retuning the PID.
    constexpr uint8_t dlpf = 0U;
    const uint8_t registers[] = {MPU_CFG_REG, SMPLRT_DIV_REG, GYRO_CONFIG_REG,
                                 ACCEL_CONFIG_REG, MPU_INTBP_CFG_REG};
    const uint8_t values[] = {dlpf, static_cast<uint8_t>(divider - 1U),
        static_cast<uint8_t>(static_cast<uint8_t>(M650_cfg.GyroRange) << 3U),
        static_cast<uint8_t>(static_cast<uint8_t>(M650_cfg.AccRange) << 3U), 0x80};
    for (unsigned i = 0U; i < sizeof(registers); ++i) {
        uint8_t actual{};
        if (!write(registers[i], values[i]) || !read(registers[i], actual) || actual != values[i]) {
            return false;
        }
    }
    uint8_t power{};
    if (!read(PWR_MGMT_1_REG, power) || power != 0x01 ||
        !read(PWR_MGMT_2_REG, power) || power != 0x00) { return false; }
    // Datasheet sensitivity in LSB/(degree/s) and LSB/g.
    constexpr double gyro_scales[] = {131.0, 65.5, 32.8, 16.4};
    constexpr double acc_scales[] = {16384.0, 8192.0, 4096.0, 2048.0};
    GyroCoefficient = gyro_scales[static_cast<uint8_t>(M650_cfg.GyroRange)];
    AccCoefficient = acc_scales[static_cast<uint8_t>(M650_cfg.AccRange)];
    vqf.resetState();
    return true;
}

bool MPU6050::getMotion(double gyro[3], double acc[3])
{
    // One coherent burst: accel, temperature (skipped), gyro from the same sample.
    uint8_t bytes[14]{};
    if (HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, bytes, sizeof(bytes),
                         MPU6050_TIME_OUT) != HAL_OK) { return false; }
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        const unsigned a = axis * 2U;
        const unsigned g = a + 8U;
        acc[axis] = static_cast<int16_t>((bytes[a] << 8U) | bytes[a + 1U]) / AccCoefficient;
        gyro[axis] = static_cast<int16_t>((bytes[g] << 8U) | bytes[g + 1U]) / GyroCoefficient +
                     M650_cfg.GyroOffset[axis];
    }
    return true;
}

bool MPU6050::getGyro(double _gyro[3])
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_StatusTypeDef Hal_bool;
    Hal_bool = HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_TIME_OUT);
    if(Hal_bool != HAL_OK){
        return false;
    }
    int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 32768 / (1000)
    _gyro[0] = (Gyro_X_RAW / GyroCoefficient) + M650_cfg.GyroOffset[0];
    _gyro[1] = (Gyro_Y_RAW / GyroCoefficient) + M650_cfg.GyroOffset[1];
    _gyro[2] = (Gyro_Z_RAW / GyroCoefficient) + M650_cfg.GyroOffset[2];
    return true;
}

bool MPU6050::getAccel(double _acc[3])
{

    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    HAL_StatusTypeDef Hal_bool{};
    Hal_bool = HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_TIME_OUT);
    if(Hal_bool != HAL_OK){
        return false;
    }
    int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    //32768 / (2.0)
    _acc[0] = Accel_X_RAW / AccCoefficient;
    _acc[1] = Accel_Y_RAW / AccCoefficient;
    _acc[2] = Accel_Z_RAW / AccCoefficient;
    return true;
}

bool MPU6050::getTemperature(float& _temp)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
    HAL_StatusTypeDef Hal_bool{};
    Hal_bool = HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, MPU6050_TIME_OUT);
    if(Hal_bool != HAL_OK){
        return false;
    }
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    _temp = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    return true;
}

void MPU6050::setGyroOffset(double &&_xg, double &&_yg, double &&_zg) {
    M650_cfg.GyroOffset[0] = _xg;
    M650_cfg.GyroOffset[1] = _yg;
    M650_cfg.GyroOffset[2] = _zg;
}

void MPU6050::setGyroOffset(double _offnum[3]) {
    setGyroOffset(std::forward<double>(_offnum[0]),std::forward<double>(_offnum[1]),std::forward<double>(_offnum[2]));
}

/**
 * @brief
 * @param _angle
 * @return
 */
bool MPU6050::getEulerAngle(MPU6050::EulerAngle &_angle) {
    double acc[3],gyro[3];
    vqf_real_t quat[4]{}; // output array for quaternion
    if(getMotion(gyro, acc))
    {
        MPU6050::DegTorad(gyro);
        MPU6050::GToMS2(acc);
        vqf.update(gyro,acc);
        vqf.getQuat6D(quat);
        QuatToEuler(quat,_angle);
        return true;
    }
    return false;
}

bool MPU6050::getEulerAngleGyro(MPU6050::EulerAngle &_angle, double *_gyro) {
    double acc[3],gyro[3];
    vqf_real_t quat[4]{}; // output array for quaternion
    if(getMotion(gyro, acc))
    {
        MPU6050::DegTorad(gyro);
        MPU6050::GToMS2(acc);
        vqf.update(gyro,acc);
        vqf.getQuat6D(quat);
        QuatToEuler(quat,_angle);
        _gyro[0] = gyro[0];
        _gyro[1] = gyro[1];
        _gyro[2] = gyro[2];
        return true;
    }
    return false;
}

bool MPU6050::getEulerAngleACC(MPU6050::EulerAngle &_angle, double *_acc) {
    double acc[3],gyro[3];
    vqf_real_t quat[4]{}; // output array for quaternion
    if(getMotion(gyro, acc))
    {
        MPU6050::DegTorad(gyro);
        MPU6050::GToMS2(acc);
        vqf.update(gyro,acc);
        vqf.getQuat6D(quat);
        QuatToEuler(quat,_angle);
        _acc[0] = acc[0];
        _acc[1] = acc[1];
        _acc[2] = acc[2];
        return true;
    }
    return false;
}

