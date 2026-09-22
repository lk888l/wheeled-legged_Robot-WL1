#include "MPU6050.h"
#include "test_check.hpp"
#include <array>
#include <cmath>

namespace {
std::array<uint8_t, 128> regs{};
bool fail_read{}, fail_write{}, corrupt_config{};
unsigned reads{}, burst_reads{}, delays{};
void raw(unsigned reg, int16_t value) {
    regs[reg] = static_cast<uint16_t>(value) >> 8;
    regs[reg + 1] = static_cast<uint8_t>(value);
}
void reset() {
    regs.fill(0); regs[0x75] = 0x68;
    fail_read = fail_write = corrupt_config = false;
    reads = burst_reads = delays = 0;
}
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t address, uint16_t reg,
                                  uint16_t, uint8_t* data, uint16_t count, uint32_t timeout) {
    CHECK(address == 0xD0 && timeout <= 5);
    ++reads;
    if (fail_read) return HAL_ERROR;
    if (reg == 0x3B && count == 14) ++burst_reads;
    std::copy_n(regs.data() + reg, count, data);
    if (corrupt_config && reg == 0x1A) data[0] ^= 1;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*, uint16_t address, uint16_t reg,
                                   uint16_t, uint8_t* data, uint16_t count, uint32_t timeout) {
    CHECK(address == 0xD0 && timeout <= 5 && count == 1);
    if (fail_write) return HAL_ERROR;
    regs[reg] = data[0];
    return HAL_OK;
}
void HAL_Delay(uint32_t delay) { delays += delay; }

int main() {
    I2C_HandleTypeDef bus;
    reset();
    MPU6050 imu(&bus);
    CHECK(imu.Init());
    CHECK(regs[0x6B] == 1 && regs[0x6C] == 0 && delays >= 130);
    // Preserve the sensor timing used before ba5b8e7: DLPF=0, 8 kHz / 10
    // gives 800 Hz register updates while attitude is consumed at 100 Hz.
    CHECK(regs[0x19] == 9 && regs[0x1A] == 0);
    CHECK(8000U / (regs[0x19] + 1U) == 800U);
    CHECK(regs[0x1B] == 16 && regs[0x1C] == 8);
    raw(0x43, 3280); raw(0x45, -3280); raw(0x47, 1640);
    double gyro[3], acc[3];
    CHECK(imu.getGyro(gyro));
    CHECK(std::abs(gyro[0] - 100.0) < 1e-8 && std::abs(gyro[1] + 100.0) < 1e-8);
    raw(0x43, 0); raw(0x45, 0); raw(0x47, 0); raw(0x3F, 8192);
    MPU6050::EulerAngle angle{};
    auto before = reads;
    CHECK(imu.getEulerAngleGyro(angle, gyro));
    CHECK(reads == before + 1 && burst_reads == 1);
    CHECK(std::abs(angle.Roll) < 0.01 && std::abs(angle.Pitch) < 0.01);
    fail_read = true;
    angle.Roll = 123;
    CHECK(!imu.getEulerAngleGyro(angle, gyro) && angle.Roll == 123);
    for (unsigned range = 0; range < 4; ++range) {
        reset();
        MPU6050 sensor(&bus, {MPU6050::GyroRange_t::G1000,
            static_cast<MPU6050::AccRange_t>(range), 100, {0, 0, 0}});
        CHECK(sensor.Init());
        raw(0x3F, static_cast<int16_t>(16384 >> range));
        CHECK(sensor.getAccel(acc) && acc[2] == 1.0);
        CHECK(sensor.getEulerAngleACC(angle, acc) && std::abs(acc[2] - 9.81) < 1e-8);
    }
    for (uint16_t rate : {0, 1, 4, 100, 333, 1000, 2000}) {
        reset();
        MPU6050 sensor(&bus, {MPU6050::GyroRange_t::G1000, MPU6050::AccRange_t::A4, rate, {}});
        CHECK(sensor.Init());
        CHECK(regs[0x19] == 1000 / std::clamp<unsigned>(rate, 4, 1000) - 1);
        CHECK(regs[0x1A] == 0);
    }
    reset(); corrupt_config = true; CHECK(!imu.Init());
    reset(); fail_write = true; CHECK(!imu.Init());
    reset(); regs[0x75] = 0; CHECK(!imu.Init());
    reset(); fail_read = true; CHECK(!imu.Init());
    // Measured board offsets exceed VQF's default 2 deg/s clip, especially Z.
    // Verify rest-bias convergence through the actual driver at a steep pitch.
    reset(); CHECK(imu.Init());
    raw(0x3B, 8144); raw(0x3F, 900);
    raw(0x43, -40); raw(0x45, -16); raw(0x47, 112);
    MPU6050::EulerAngle settled{};
    for (unsigned i = 0; i < 1500; ++i) {
        CHECK(imu.getEulerAngle(angle));
        if (i == 1399) settled = angle;
    }
    CHECK(std::abs(angle.Roll - settled.Roll) < 0.1);
    CHECK(std::abs(angle.Pitch - settled.Pitch) < 0.05);
    CHECK(std::abs(angle.Yaw - settled.Yaw) < 0.1);
    CHECK(imu.getEulerAngleGyro(angle, gyro));
    for (double rate : gyro) CHECK(std::abs(rate) < 0.002); // rad/s, learned bias removed.
    // The low-level diagnostic API still reports degrees/s before VQF correction.
    CHECK(imu.getGyro(gyro));
    CHECK(std::abs(gyro[2] - 112.0 / 32.8) < 1e-8);
    // Feedback does not erase real movement together with the learned zero bias.
    raw(0x45, 3264); // old -16 count Y bias plus +100 deg/s motion.
    CHECK(imu.getEulerAngleGyro(angle, gyro));
    CHECK(std::abs(gyro[1] - 1.7453292519943295) < 0.005);
    std::puts("PASS: MPU6050 clock, DLPF, divider, ranges, coherent sample and I/O failures");
}
