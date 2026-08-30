#include "MPU6050.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

struct RegisterWrite
{
    std::uint16_t address{};
    std::uint8_t value{};
    std::uint32_t timeout{};
};

std::array<RegisterWrite, 16> writes{};
std::size_t write_count = 0;
std::array<std::uint8_t, 14> burst{};
std::uint16_t last_read_address = 0;
std::uint16_t last_read_size = 0;
std::uint32_t last_read_timeout = 0;
bool fail_burst_read = false;

void setBurstWord(std::size_t offset, std::int16_t value)
{
    const auto bits = static_cast<std::uint16_t>(value);
    burst[offset] = static_cast<std::uint8_t>(bits >> 8U);
    burst[offset + 1] = static_cast<std::uint8_t>(bits & 0xFFU);
}

bool nearlyEqual(double left, double right, double tolerance = 1e-9)
{
    return std::abs(left - right) <= tolerance;
}

void acquireStableGravity(MPU6050& imu,
                          const std::int16_t accel[3],
                          MPU6050::Sample& sample)
{
    const std::int16_t zero_gyro[3] = {0, 0, 0};
    for (std::uint32_t index = 0; index < 30U; ++index)
    {
        const bool valid =
            imu.processRawSample(accel, zero_gyro, sample, 0, true);
        if (index + 1U < 30U)
        {
            assert(!valid);
            assert(!sample.valid);
            assert(!sample.accel_trusted);
        }
        else
        {
            assert(valid);
            assert(sample.valid);
            assert(sample.accel_trusted);
        }
    }
}

const RegisterWrite& findWrite(std::uint16_t address)
{
    for (std::size_t index = 0; index < write_count; ++index)
    {
        if (writes[index].address == address)
        {
            return writes[index];
        }
    }
    assert(false && "expected MPU6050 register write was not observed");
    return writes[0];
}

} // namespace

extern "C" HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*,
                                                std::uint16_t,
                                                std::uint16_t memory_address,
                                                std::uint16_t,
                                                std::uint8_t* data,
                                                std::uint16_t size,
                                                std::uint32_t timeout)
{
    last_read_address = memory_address;
    last_read_size = size;
    last_read_timeout = timeout;

    if (memory_address == 0x75 && size == 1)
    {
        data[0] = 0x68;
        return HAL_OK;
    }
    if (memory_address == 0x3B && size == burst.size() && !fail_burst_read)
    {
        std::memcpy(data, burst.data(), burst.size());
        return HAL_OK;
    }
    return HAL_ERROR;
}

extern "C" HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*,
                                                 std::uint16_t,
                                                 std::uint16_t memory_address,
                                                 std::uint16_t,
                                                 std::uint8_t* data,
                                                 std::uint16_t size,
                                                 std::uint32_t timeout)
{
    assert(size == 1);
    assert(write_count < writes.size());
    writes[write_count++] = RegisterWrite{memory_address, data[0], timeout};
    return HAL_OK;
}

int main()
{
    assert(nearlyEqual(MPU6050::gyroCoefficientForRange(MPU6050::GyroRange_t::G1000),
                       32.768));
    assert(nearlyEqual(MPU6050::accelCoefficientForRange(MPU6050::AccRange_t::A8),
                       4096.0));

    I2C_HandleTypeDef i2c{};
    MPU6050 imu(&i2c,
                {MPU6050::GyroRange_t::G1000,
                 MPU6050::AccRange_t::A4,
                 500,
                 {0.0, 0.0, 0.0}});
    assert(imu.Init());

    // Legacy caller settings are promoted to the fixed robust sensor profile.
    assert(write_count == 6);
    assert(findWrite(0x6B).value == 0x01); // retain gyroscope PLL
    assert(findWrite(0x19).value == 9);    // 1 kHz / (9 + 1) = 100 Hz
    assert(findWrite(0x1A).value == 3);    // DLPF_CFG=3
    assert(findWrite(0x1C).value == 0x10); // A8
    assert(findWrite(0x1B).value == 0x18); // G2000
    for (std::size_t index = 0; index < write_count; ++index)
    {
        assert(writes[index].timeout == 2);
    }

    // One coherent 14-byte burst: +1g Z, 0 deg/s, 36.53 C.
    burst.fill(0);
    setBurstWord(4, 4096);
    MPU6050::Sample sample{};
    for (std::uint32_t index = 0; index < 30U; ++index)
    {
        assert(imu.getSample(sample, true));
        assert(sample.valid == (index + 1U == 30U));
    }
    assert(last_read_address == 0x3B);
    assert(last_read_size == 14);
    assert(last_read_timeout == 2);
    assert(sample.valid);
    assert(sample.accel_trusted);
    assert(sample.saturation_flags == 0);
    assert(nearlyEqual(sample.accel[2], 9.81, 1e-12));

    // G2000 and A8 physical-unit conversion used by legacy accessors.
    setBurstWord(4, 4096);
    setBurstWord(8, 16384);
    double accel_g[3]{};
    double gyro_degrees_per_second[3]{};
    assert(imu.getAccel(accel_g));
    assert(imu.getGyro(gyro_degrees_per_second));
    assert(nearlyEqual(accel_g[2], 1.0));
    assert(nearlyEqual(gyro_degrees_per_second[0], 1000.0));

    // New samples expose SI units, but the legacy Euler+gyro API must keep
    // deg/s because its LQR caller performs DegTorad() itself.
    assert(imu.getSample(sample));
    assert(nearlyEqual(sample.gyro[0], MPU6050::DegTorad(1000.0), 1e-12));
    MPU6050::EulerAngle legacy_angle{};
    double legacy_gyro[3]{};
    assert(imu.getEulerAngleGyro(legacy_angle, legacy_gyro));
    assert(nearlyEqual(legacy_gyro[0], 1000.0, 1e-9));

    // A dynamic acceleration outside 0.8g..1.2g is rejected, while an already
    // initialized attitude remains valid through gyro-only propagation.
    const std::int16_t dynamic_accel[3] = {4096, 0, 4096};
    const std::int16_t zero_gyro[3] = {0, 0, 0};
    const std::int16_t one_g_accel[3] = {0, 0, 4096};

    // The legacy attitude API remains usable on a fresh instance: it explicitly
    // opts into stationary acquisition, while the new getSample default does not.
    MPU6050 fresh_legacy_imu(&i2c);
    burst.fill(0);
    setBurstWord(4, 4096);
    for (std::uint32_t index = 0; index < 29U; ++index)
    {
        assert(!fresh_legacy_imu.getEulerAngle(legacy_angle));
    }
    assert(fresh_legacy_imu.getEulerAngle(legacy_angle));

    // After the first rest norm has been learned, the no-hint legacy wrapper
    // must not authorize a hard reset from a stable dynamic acceleration.
    fresh_legacy_imu.resetFusion();
    burst.fill(0);
    setBurstWord(0, 1229); // 0.300 g lateral
    setBurstWord(4, 3907); // 0.954 g vertical; combined norm is about 1 g
    for (std::uint32_t index = 0; index < 40U; ++index)
    {
        assert(!fresh_legacy_imu.getEulerAngle(legacy_angle));
    }
    burst.fill(0);
    setBurstWord(4, 4096);
    for (std::uint32_t index = 0; index < 29U; ++index)
    {
        assert(!fresh_legacy_imu.getEulerAngle(legacy_angle, true));
    }
    assert(fresh_legacy_imu.getEulerAngle(legacy_angle, true));

    assert(imu.processRawSample(dynamic_accel, zero_gyro, sample));
    assert(!sample.accel_trusted);
    assert(sample.valid);

    // Magnitude alone cannot distinguish tilt from a sustained horizontal
    // acceleration: 0.5 g lateral + 1 g vertical has norm 1.118 g. The
    // direction innovation gate must keep it out of the gravity correction.
    MPU6050 direction_gate_imu(&i2c);
    MPU6050::Sample direction_sample{};
    acquireStableGravity(direction_gate_imu, one_g_accel, direction_sample);
    const std::int16_t lateral_half_g[3] = {2048, 0, 4096};
    for (std::uint32_t index = 0; index < 100U; ++index)
    {
        assert(direction_gate_imu.processRawSample(
            lateral_half_g, zero_gyro, direction_sample));
        assert(!direction_sample.accel_trusted);
    }
    assert(std::fabs(direction_sample.angle.Pitch) < 2.0);

    // A slow acceleration ramp must not evade the innovation gate by letting
    // the attitude follow it. Once the learned rest-norm tolerance is crossed,
    // the accelerometer is rejected and the apparent tilt stays bounded.
    MPU6050 ramp_gate_imu(&i2c);
    MPU6050::Sample ramp_sample{};
    acquireStableGravity(ramp_gate_imu, one_g_accel, ramp_sample);
    double maximum_ramp_pitch = 0.0;
    for (std::uint32_t index = 1; index <= 200U; ++index)
    {
        const double lateral_g = 0.5 * static_cast<double>(index) / 200.0;
        const std::int16_t ramp_accel[3] = {
            static_cast<std::int16_t>(std::lround(lateral_g * 4096.0)),
            0,
            4096,
        };
        assert(ramp_gate_imu.processRawSample(
            ramp_accel, zero_gyro, ramp_sample, 0, false));
        maximum_ramp_pitch = std::max(
            maximum_ramp_pitch, std::fabs(ramp_sample.angle.Pitch));
    }
    assert(!ramp_sample.accel_trusted);
    assert(maximum_ramp_pitch < 15.0);

    // A smaller 0.3 g lateral component remains inside the broad magnitude
    // window (1.044 g), but exceeds the 12-degree direction innovation limit
    // and also differs from the learned rest norm by more than 0.03 g.
    const std::int16_t lateral_point_three_g[3] = {1229, 0, 4096};
    for (std::uint32_t index = 0; index < 100U; ++index)
    {
        assert(direction_gate_imu.processRawSample(
            lateral_point_three_g, zero_gyro, direction_sample, 0, true));
        assert(!direction_sample.accel_trusted);
    }
    assert(std::fabs(direction_sample.angle.Pitch) < 2.0);
    for (std::uint32_t index = 0; index < 50U; ++index)
    {
        assert(direction_gate_imu.processRawSample(
            one_g_accel, zero_gyro, direction_sample));
        assert(direction_sample.accel_trusted);
    }
    assert(std::fabs(direction_sample.angle.Pitch) < 2.0);

    // Reproduce a large motion: integrate about 45 degrees while acceleration
    // is deliberately rejected, then verify that trusted gravity brings pitch
    // back promptly with the 0.5 s correction time constant.
    MPU6050 recovery_imu(&i2c);
    MPU6050::Sample recovery_sample{};
    acquireStableGravity(recovery_imu, one_g_accel, recovery_sample);
    const std::int16_t fast_pitch_gyro[3] = {0, 1475, 0}; // about 90 deg/s at G2000
    for (std::uint32_t index = 0; index < 50U; ++index)
    {
        assert(recovery_imu.processRawSample(
            dynamic_accel, fast_pitch_gyro, recovery_sample));
    }
    assert(std::fabs(recovery_sample.angle.Pitch) > 30.0);
    bool hard_reacquired = false;
    std::uint32_t reacquire_sample_count = 0;
    for (std::uint32_t index = 0; index < 200U; ++index)
    {
        assert(recovery_imu.processRawSample(
            one_g_accel, zero_gyro, recovery_sample, 0, true));
        if (recovery_sample.accel_trusted && !hard_reacquired)
        {
            hard_reacquired = true;
            reacquire_sample_count = index + 1U;
        }
    }
    assert(hard_reacquired);
    assert(reacquire_sample_count <= 30U);
    assert(std::fabs(recovery_sample.angle.Pitch) < 5.0);

    // Raw near-full-scale values are surfaced. Gyro saturation invalidates
    // attitude, whereas an accelerometer impact is gated but gyro propagation
    // can remain valid.
    const std::int16_t saturated_gyro[3] = {32000, 0, 0};
    assert(!imu.processRawSample(one_g_accel, saturated_gyro, sample));
    assert((sample.saturation_flags & MPU6050::GyroXRawSaturated) != 0U);
    assert(!sample.valid);

    const std::int16_t saturated_accel[3] = {32000, 0, 0};
    // Gyro clipping resets fusion history, so an untrusted impact cannot by
    // itself re-enable attitude output.
    assert(!imu.processRawSample(saturated_accel, zero_gyro, sample));
    assert((sample.saturation_flags & MPU6050::AccelXRawSaturated) != 0U);
    assert(!sample.accel_trusted);
    assert(!sample.valid);

    // Gravity reacquisition restores validity; later dynamic acceleration is
    // again handled by bounded gyro-only propagation.
    acquireStableGravity(imu, one_g_accel, sample);
    assert(imu.processRawSample(saturated_accel, zero_gyro, sample));
    assert((sample.saturation_flags & MPU6050::AccelXRawSaturated) != 0U);
    assert(!sample.accel_trusted);
    assert(sample.valid);

    // Reset/startup cannot be initialized by the first plausible dynamic
    // sample. A 1.118 g half-g lateral vector is outside the initial candidate
    // window and remains invalid even when presented repeatedly.
    MPU6050 reset_dynamic_imu(&i2c);
    MPU6050::Sample reset_dynamic_sample{};
    for (std::uint32_t index = 0; index < 40U; ++index)
    {
        assert(!reset_dynamic_imu.processRawSample(
            lateral_half_g, zero_gyro, reset_dynamic_sample, 0, true));
        assert(!reset_dynamic_sample.accel_trusted);
    }

    // A calibrated stationary sensor reading 1.051 g is accepted in 300 ms and
    // becomes its own rest-norm reference rather than locking out forever.
    MPU6050 calibrated_norm_imu(&i2c);
    MPU6050::Sample calibrated_norm_sample{};
    const std::int16_t calibrated_one_g[3] = {0, 0, 4305};
    acquireStableGravity(calibrated_norm_imu,
                         calibrated_one_g,
                         calibrated_norm_sample);
    assert(calibrated_norm_sample.valid);

    // A small static gyro residual just above the old 3 deg/s cutoff must not
    // permanently deadlock startup/recovery. The external stationary hint and
    // the 300 ms stable-gravity window still bound this path.
    MPU6050 biased_gyro_imu(&i2c);
    MPU6050::Sample biased_gyro_sample{};
    const std::int16_t small_static_gyro_bias[3] = {50, 0, 0}; // about 3.05 deg/s
    for (std::uint32_t index = 0; index < 30U; ++index)
    {
        const bool valid = biased_gyro_imu.processRawSample(
            one_g_accel, small_static_gyro_bias, biased_gyro_sample, 0, true);
        assert(valid == (index + 1U == 30U));
    }
    assert(biased_gyro_sample.valid);

    // A successful I2C transfer is distinct from fusion validity.
    burst.fill(0);
    setBurstWord(4, 4096);
    setBurstWord(8, 32000);
    assert(imu.getSample(sample));
    assert(!sample.valid);
    assert((sample.saturation_flags & MPU6050::GyroXRawSaturated) != 0U);

    // A slightly non-unit quaternion must not push asin outside [-1, 1].
    const double non_unit_quaternion[4] = {1.0, 0.0, 0.5000001, 0.0};
    MPU6050::EulerAngle angle{};
    MPU6050::QuatToEuler(non_unit_quaternion, angle);
    assert(std::isfinite(angle.Pitch));
    assert(nearlyEqual(angle.Pitch, 90.0, 1e-9));

    // Transport failure has a separate return value and clears sample.valid.
    fail_burst_read = true;
    sample.valid = true;
    assert(!imu.getSample(sample));
    assert(!sample.valid);
    return 0;
}
