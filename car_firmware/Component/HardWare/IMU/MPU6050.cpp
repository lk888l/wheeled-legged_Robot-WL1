/********************************************************************************
  * @file           : MPU6050.cpp
  * @author         : Luka
  * @brief          : MPU6050 sampling and VQF attitude fusion
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/

#include "MPU6050.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <utility>

namespace {

constexpr std::uint8_t kAccelerometerSaturationMask =
    MPU6050::AccelXRawSaturated | MPU6050::AccelYRawSaturated
    | MPU6050::AccelZRawSaturated;
constexpr std::uint8_t kGyroscopeSaturationMask =
    MPU6050::GyroXRawSaturated | MPU6050::GyroYRawSaturated
    | MPU6050::GyroZRawSaturated;

std::int16_t decodeSigned16(const std::uint8_t* bytes)
{
    const auto value = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
    return static_cast<std::int16_t>(value);
}

bool isFinite(const MPU6050::EulerAngle& angle)
{
    return std::isfinite(angle.Roll) && std::isfinite(angle.Pitch)
        && std::isfinite(angle.Yaw);
}

} // namespace

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c)
    : MPU6050(hi2c, InitConfig_t{})
{
}

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, MPU6050::InitConfig_t config)
    : Hi2c(hi2c),
      M650_cfg(config),
      vqf(1.0 / static_cast<double>(FUSION_SAMPLE_RATE_HZ))
{
    // The balancing and jump estimator uses one known timing/range profile.
    // Keep accepting the legacy aggregate config so existing callers compile,
    // while enforcing the robust on-wire sensor configuration here.
    M650_cfg.GyroRange = GyroRange_t::G2000;
    M650_cfg.AccRange = AccRange_t::A8;
    M650_cfg.SampleRate = FUSION_SAMPLE_RATE_HZ;

    GyroCoefficient = gyroCoefficientForRange(M650_cfg.GyroRange);
    AccCoefficient = accelCoefficientForRange(M650_cfg.AccRange);
    vqf.setTauAcc(VQF_ACCEL_TIME_CONSTANT_SECONDS);
}

double MPU6050::gyroCoefficientForRange(GyroRange_t range)
{
    switch (range)
    {
    case GyroRange_t::G250:
        return 32768.0 / 250.0;
    case GyroRange_t::G500:
        return 32768.0 / 500.0;
    case GyroRange_t::G1000:
        return 32768.0 / 1000.0;
    case GyroRange_t::G2000:
        return 32768.0 / 2000.0;
    }
    return 32768.0 / 2000.0;
}

double MPU6050::accelCoefficientForRange(AccRange_t range)
{
    switch (range)
    {
    case AccRange_t::A2:
        return 32768.0 / 2.0;
    case AccRange_t::A4:
        return 32768.0 / 4.0;
    case AccRange_t::A8:
        return 32768.0 / 8.0;
    case AccRange_t::A16:
        return 32768.0 / 16.0;
    }
    return 32768.0 / 8.0;
}

bool MPU6050::Init()
{
    if (Hi2c == nullptr)
    {
        return false;
    }

    std::uint8_t identity = 0;
    if (HAL_I2C_Mem_Read(Hi2c,
                         MPU6050_ADDR,
                         WHO_AM_I_REG,
                         1,
                         &identity,
                         1,
                         MPU6050_TIME_OUT)
            != HAL_OK
        || identity != 0x68)
    {
        return false;
    }

    const auto writeRegister = [this](std::uint8_t address, std::uint8_t value) {
        return HAL_I2C_Mem_Write(Hi2c,
                                 MPU6050_ADDR,
                                 address,
                                 1,
                                 &value,
                                 1,
                                 MPU6050_TIME_OUT)
            == HAL_OK;
    };

    // Wake the device and retain the recommended X-axis gyroscope PLL clock.
    if (!writeRegister(PWR_MGMT_1_REG, 0x01))
    {
        return false;
    }

    // DLPF_CFG=3 selects a 1 kHz source. DIV=9 therefore yields exactly 100 Hz.
    constexpr std::uint8_t sampleRateDivider =
        (1000U / FUSION_SAMPLE_RATE_HZ) - 1U;
    if (!writeRegister(SMPLRT_DIV_REG, sampleRateDivider)
        || !writeRegister(MPU_CFG_REG, DLPF_CONFIG)
        || !writeRegister(ACCEL_CONFIG_REG,
                          static_cast<std::uint8_t>(M650_cfg.AccRange) << 3U)
        || !writeRegister(GYRO_CONFIG_REG,
                          static_cast<std::uint8_t>(M650_cfg.GyroRange) << 3U)
        || !writeRegister(MPU_INTBP_CFG_REG, 0x80))
    {
        return false;
    }

    GyroCoefficient = gyroCoefficientForRange(M650_cfg.GyroRange);
    AccCoefficient = accelCoefficientForRange(M650_cfg.AccRange);
    has_trusted_accel_ = false;
    accel_direction_trusted_ = false;
    has_accel_norm_reference_ = false;
    accel_norm_reference_g_ = 1.0;
    resetAccelCandidate();
    vqf.resetState();
    return true;
}

bool MPU6050::readRawSample(RawSample& sample)
{
    std::uint8_t data[14]{};
    if (Hi2c == nullptr
        || HAL_I2C_Mem_Read(Hi2c,
                            MPU6050_ADDR,
                            ACCEL_XOUT_H_REG,
                            1,
                            data,
                            sizeof(data),
                            MPU6050_TIME_OUT)
            != HAL_OK)
    {
        return false;
    }

    sample.accel[0] = decodeSigned16(data + 0);
    sample.accel[1] = decodeSigned16(data + 2);
    sample.accel[2] = decodeSigned16(data + 4);
    sample.temperature = decodeSigned16(data + 6);
    sample.gyro[0] = decodeSigned16(data + 8);
    sample.gyro[1] = decodeSigned16(data + 10);
    sample.gyro[2] = decodeSigned16(data + 12);
    return true;
}

void MPU6050::convertRawGyro(const std::int16_t raw[3],
                             double gyro_degrees_per_second[3]) const
{
    for (std::size_t index = 0; index < 3; ++index)
    {
        gyro_degrees_per_second[index] =
            (static_cast<double>(raw[index]) / GyroCoefficient)
            + M650_cfg.GyroOffset[index];
    }
}

void MPU6050::convertRawAccel(const std::int16_t raw[3], double accel_g[3]) const
{
    for (std::size_t index = 0; index < 3; ++index)
    {
        accel_g[index] = static_cast<double>(raw[index]) / AccCoefficient;
    }
}

std::uint8_t MPU6050::getSaturationFlags(const std::int16_t raw_accel[3],
                                         const std::int16_t raw_gyro[3])
{
    const auto nearFullScale = [](std::int16_t value) {
        return std::abs(static_cast<std::int32_t>(value))
            >= RAW_SATURATION_THRESHOLD;
    };

    std::uint8_t flags = 0;
    if (nearFullScale(raw_accel[0]))
    {
        flags |= AccelXRawSaturated;
    }
    if (nearFullScale(raw_accel[1]))
    {
        flags |= AccelYRawSaturated;
    }
    if (nearFullScale(raw_accel[2]))
    {
        flags |= AccelZRawSaturated;
    }
    if (nearFullScale(raw_gyro[0]))
    {
        flags |= GyroXRawSaturated;
    }
    if (nearFullScale(raw_gyro[1]))
    {
        flags |= GyroYRawSaturated;
    }
    if (nearFullScale(raw_gyro[2]))
    {
        flags |= GyroZRawSaturated;
    }
    return flags;
}

void MPU6050::resetAccelCandidate()
{
    accel_reacquire_samples_ = 0;
    accel_candidate_norm_sum_g_ = 0.0;
    std::fill(std::begin(accel_candidate_direction_sum_),
              std::end(accel_candidate_direction_sum_),
              0.0);
    std::fill(std::begin(previous_accel_candidate_direction_),
              std::end(previous_accel_candidate_direction_),
              0.0);
}

bool MPU6050::tryStableAccelReacquisition(const double accel_g[3],
                                          double accel_norm_g,
                                          double gyro_norm_rad_per_second,
                                          bool stationary_hint)
{
    const double reference_tolerance = std::max(
        ACCEL_REFERENCE_TOLERANCE_G,
        has_accel_norm_reference_ ? 0.03 * accel_norm_reference_g_ : 0.0);
    const bool magnitude_candidate = has_accel_norm_reference_
        ? std::fabs(accel_norm_g - accel_norm_reference_g_) <= reference_tolerance
        : accel_norm_g >= MIN_INITIAL_ACCEL_G && accel_norm_g <= MAX_INITIAL_ACCEL_G;
    if (!stationary_hint || !magnitude_candidate
        || gyro_norm_rad_per_second > MAX_REACQUIRE_GYRO_RAD_PER_SECOND
        || accel_norm_g <= 0.0)
    {
        resetAccelCandidate();
        return false;
    }

    double direction[3] = {
        accel_g[0] / accel_norm_g,
        accel_g[1] / accel_norm_g,
        accel_g[2] / accel_norm_g,
    };
    const auto beginCandidate = [&]() {
        accel_reacquire_samples_ = 1;
        accel_candidate_norm_sum_g_ = accel_norm_g;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            accel_candidate_direction_sum_[axis] = direction[axis];
            previous_accel_candidate_direction_[axis] = direction[axis];
        }
    };
    if (accel_reacquire_samples_ == 0U)
    {
        beginCandidate();
        return false;
    }

    double mean_direction[3] = {
        accel_candidate_direction_sum_[0],
        accel_candidate_direction_sum_[1],
        accel_candidate_direction_sum_[2],
    };
    const double mean_direction_norm = std::sqrt(
        mean_direction[0] * mean_direction[0]
        + mean_direction[1] * mean_direction[1]
        + mean_direction[2] * mean_direction[2]);
    if (mean_direction_norm <= 0.0)
    {
        beginCandidate();
        return false;
    }
    for (double& component : mean_direction)
    {
        component /= mean_direction_norm;
    }
    const double previous_agreement =
        direction[0] * previous_accel_candidate_direction_[0]
        + direction[1] * previous_accel_candidate_direction_[1]
        + direction[2] * previous_accel_candidate_direction_[2];
    const double mean_agreement = direction[0] * mean_direction[0]
        + direction[1] * mean_direction[1] + direction[2] * mean_direction[2];
    const double mean_norm = accel_candidate_norm_sum_g_
        / static_cast<double>(accel_reacquire_samples_);
    if (previous_agreement < ACCEL_CANDIDATE_STEP_COS
        || mean_agreement < ACCEL_CANDIDATE_MEAN_COS
        || std::fabs(accel_norm_g - mean_norm) > ACCEL_CANDIDATE_NORM_STEP_G)
    {
        // The current sample may be the first member of a new stable window.
        beginCandidate();
        return false;
    }

    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        accel_candidate_direction_sum_[axis] += direction[axis];
        previous_accel_candidate_direction_[axis] = direction[axis];
    }
    accel_candidate_norm_sum_g_ += accel_norm_g;
    if (accel_reacquire_samples_ < ACCEL_REACQUIRE_SAMPLES)
    {
        ++accel_reacquire_samples_;
    }
    if (accel_reacquire_samples_ < ACCEL_REACQUIRE_SAMPLES)
    {
        return false;
    }

    const double accepted_norm = accel_candidate_norm_sum_g_
        / static_cast<double>(accel_reacquire_samples_);
    double accepted_direction[3] = {
        accel_candidate_direction_sum_[0],
        accel_candidate_direction_sum_[1],
        accel_candidate_direction_sum_[2],
    };
    const double accepted_direction_norm = std::sqrt(
        accepted_direction[0] * accepted_direction[0]
        + accepted_direction[1] * accepted_direction[1]
        + accepted_direction[2] * accepted_direction[2]);
    for (double& component : accepted_direction)
    {
        component /= accepted_direction_norm;
    }

    // A forced re-acquisition deliberately resets attitude and accelerometer LP
    // history, but retains VQF's learned gyro bias and uncertainty.
    vqf_real_t gyro_bias[3]{};
    const vqf_real_t gyro_bias_sigma = vqf.getBiasEstimate(gyro_bias);
    vqf.resetState();
    vqf.setBiasEstimate(gyro_bias, gyro_bias_sigma);
    vqf_real_t accepted_acceleration[3] = {
        accepted_direction[0] * accepted_norm * 9.81,
        accepted_direction[1] * accepted_norm * 9.81,
        accepted_direction[2] * accepted_norm * 9.81,
    };
    vqf.updateAcc(accepted_acceleration);
    if (!has_accel_norm_reference_)
    {
        accel_norm_reference_g_ = accepted_norm;
        has_accel_norm_reference_ = true;
    }
    has_trusted_accel_ = true;
    accel_direction_trusted_ = true;
    resetAccelCandidate();
    return true;
}

bool MPU6050::getGyro(double gyro[3])
{
    RawSample raw{};
    if (!readRawSample(raw))
    {
        return false;
    }
    convertRawGyro(raw.gyro, gyro);
    return true;
}

bool MPU6050::getAccel(double accel[3])
{
    RawSample raw{};
    if (!readRawSample(raw))
    {
        return false;
    }
    convertRawAccel(raw.accel, accel);
    return true;
}

bool MPU6050::getTemperature(float& temperature)
{
    RawSample raw{};
    if (!readRawSample(raw))
    {
        return false;
    }
    temperature = static_cast<float>(raw.temperature) / 340.0F + 36.53F;
    return true;
}

void MPU6050::setGyroOffset(double&& x, double&& y, double&& z)
{
    M650_cfg.GyroOffset[0] = x;
    M650_cfg.GyroOffset[1] = y;
    M650_cfg.GyroOffset[2] = z;
}

void MPU6050::setGyroOffset(double offset[3])
{
    setGyroOffset(std::forward<double>(offset[0]),
                  std::forward<double>(offset[1]),
                  std::forward<double>(offset[2]));
}

bool MPU6050::processRawSample(const std::int16_t raw_accel[3],
                               const std::int16_t raw_gyro[3],
                               Sample& sample,
                               std::int16_t raw_temperature,
                               bool stationary_hint)
{
    sample = Sample{};
    std::copy(raw_accel, raw_accel + 3, sample.raw_accel);
    std::copy(raw_gyro, raw_gyro + 3, sample.raw_gyro);
    sample.temperature_c = static_cast<float>(raw_temperature) / 340.0F + 36.53F;
    sample.saturation_flags = getSaturationFlags(raw_accel, raw_gyro);

    double accel_g[3]{};
    double gyro_degrees_per_second[3]{};
    convertRawAccel(raw_accel, accel_g);
    convertRawGyro(raw_gyro, gyro_degrees_per_second);

    for (std::size_t index = 0; index < 3; ++index)
    {
        sample.accel[index] = accel_g[index] * 9.81;
        sample.gyro[index] = DegTorad(gyro_degrees_per_second[index]);
    }

    const double accelNormG = std::sqrt(
        accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1]
        + accel_g[2] * accel_g[2]);
    const double tracking_reference_tolerance = std::max(
        ACCEL_REFERENCE_TOLERANCE_G,
        has_accel_norm_reference_ ? 0.03 * accel_norm_reference_g_ : 0.0);
    const bool accelMatchesRestNorm = !has_accel_norm_reference_
        || std::fabs(accelNormG - accel_norm_reference_g_)
            <= tracking_reference_tolerance;
    const bool accelMagnitudeTrusted =
        (sample.saturation_flags & kAccelerometerSaturationMask) == 0U
        && accelNormG >= MIN_TRUSTED_ACCEL_G
        && accelNormG <= MAX_TRUSTED_ACCEL_G
        && accelMatchesRestNorm;
    sample.accel_trusted = false;
    const bool gyroSaturated =
        (sample.saturation_flags & kGyroscopeSaturationMask) != 0U;
    vqf_real_t estimated_gyro_bias[3]{};
    static_cast<void>(vqf.getBiasEstimate(estimated_gyro_bias));
    const double gyroReacquisitionNorm = std::sqrt(
        (sample.gyro[0] - estimated_gyro_bias[0])
            * (sample.gyro[0] - estimated_gyro_bias[0])
        + (sample.gyro[1] - estimated_gyro_bias[1])
            * (sample.gyro[1] - estimated_gyro_bias[1])
        + (sample.gyro[2] - estimated_gyro_bias[2])
            * (sample.gyro[2] - estimated_gyro_bias[2]));
    if (gyroSaturated)
    {
        // A clipped angular rate cannot be integrated into a recoverable pose.
        // Re-acquire gravity on subsequent non-saturated, trusted samples.
        resetFusion();
    }
    else
    {
        vqf.updateGyr(sample.gyro);
        bool hard_reacquired = false;
        if (accelMagnitudeTrusted && has_trusted_accel_)
        {
            vqf_real_t quaternion[4]{};
            vqf.getQuat6D(quaternion);
            // VQF's quaternion rotates sensor vectors into earth coordinates.
            // R^T*[0,0,1] is therefore predicted +gravity in sensor axes.
            const double predictedGravity[3] = {
                2.0 * (quaternion[3] * quaternion[1]
                       - quaternion[0] * quaternion[2]),
                2.0 * (quaternion[0] * quaternion[1]
                       + quaternion[3] * quaternion[2]),
                1.0 - (2.0 * quaternion[1] * quaternion[1])
                    - (2.0 * quaternion[2] * quaternion[2]),
            };
            const double inverseNorm = 1.0 / accelNormG;
            const double directionAgreement = std::clamp(
                (predictedGravity[0] * accel_g[0]
                 + predictedGravity[1] * accel_g[1]
                 + predictedGravity[2] * accel_g[2])
                    * inverseNorm,
                -1.0,
                1.0);
            if (accel_direction_trusted_)
            {
                if (directionAgreement < ACCEL_DIRECTION_REJECT_COS)
                {
                    accel_direction_trusted_ = false;
                    accel_reacquire_samples_ = 0;
                }
            }
            else if (directionAgreement >= ACCEL_DIRECTION_ACCEPT_COS)
            {
                accel_direction_trusted_ = true;
                resetAccelCandidate();
            }
        }

        if (accelMagnitudeTrusted && has_trusted_accel_
            && accel_direction_trusted_)
        {
            sample.accel_trusted = true;
            resetAccelCandidate();
        }
        else
        {
            hard_reacquired = tryStableAccelReacquisition(
                accel_g, accelNormG, gyroReacquisitionNorm, stationary_hint);
            sample.accel_trusted = hard_reacquired;
        }
        if (sample.accel_trusted && !hard_reacquired)
        {
            vqf.updateAcc(sample.accel);
        }
        has_trusted_accel_ = has_trusted_accel_ || sample.accel_trusted;
    }

    vqf_real_t quaternion[4]{};
    vqf.getQuat6D(quaternion);
    QuatToEuler(quaternion, sample.angle);

    sample.valid = has_trusted_accel_ && !gyroSaturated && isFinite(sample.angle);
    return sample.valid;
}

bool MPU6050::getSample(Sample& sample, bool stationary_hint)
{
    RawSample raw{};
    if (!readRawSample(raw))
    {
        sample = Sample{};
        return false;
    }
    static_cast<void>(
        processRawSample(raw.accel, raw.gyro, sample, raw.temperature, stationary_hint));
    return true;
}

void MPU6050::resetFusion()
{
    vqf_real_t gyro_bias[3]{};
    const vqf_real_t gyro_bias_sigma = vqf.getBiasEstimate(gyro_bias);
    vqf.resetState();
    vqf.setBiasEstimate(gyro_bias, gyro_bias_sigma);
    has_trusted_accel_ = false;
    accel_direction_trusted_ = false;
    resetAccelCandidate();
}

bool MPU6050::getEulerAngle(EulerAngle& angle)
{
    // Preserve automatic acquisition only for a genuinely fresh instance.
    // Once a rest-norm reference exists, a reset/reacquisition must be
    // authorized explicitly by a caller that can also observe wheel/command
    // state; a stable dynamic acceleration must not masquerade as gravity.
    return getEulerAngle(angle, !has_accel_norm_reference_);
}

bool MPU6050::getEulerAngle(EulerAngle& angle, bool stationary_hint)
{
    Sample sample{};
    if (!getSample(sample, stationary_hint) || !sample.valid)
    {
        return false;
    }
    angle = sample.angle;
    return true;
}

bool MPU6050::getEulerAngleGyro(EulerAngle& angle, double gyro[3])
{
    return getEulerAngleGyro(angle, gyro, !has_accel_norm_reference_);
}

bool MPU6050::getEulerAngleGyro(EulerAngle& angle,
                                double gyro[3],
                                bool stationary_hint)
{
    Sample sample{};
    if (!getSample(sample, stationary_hint) || !sample.valid)
    {
        return false;
    }
    angle = sample.angle;
    // Preserve the legacy unit contract: existing LQR callers convert this
    // deg/s value with DegTorad(). Sample::gyro intentionally stays in rad/s.
    for (std::size_t index = 0; index < 3; ++index)
    {
        gyro[index] = sample.gyro[index] / DEG_TO_RAD_COE;
    }
    return true;
}

bool MPU6050::getEulerAngleACC(EulerAngle& angle, double accel[3])
{
    return getEulerAngleACC(angle, accel, !has_accel_norm_reference_);
}

bool MPU6050::getEulerAngleACC(EulerAngle& angle,
                               double accel[3],
                               bool stationary_hint)
{
    Sample sample{};
    if (!getSample(sample, stationary_hint) || !sample.valid)
    {
        return false;
    }
    angle = sample.angle;
    std::copy(sample.accel, sample.accel + 3, accel);
    return true;
}
