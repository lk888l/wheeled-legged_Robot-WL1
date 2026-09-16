#include <cmath>
#include "MPU6050.h"
#include "test_check.hpp"

int main()
{
    I2C_HandleTypeDef device;
    device.registers[0x75] = 0x68;
    device.registers[0x3f] = 0x20; // +1g in Z at +/-4g full scale.
    device.registers[0x46] = 33; // ~1 deg/s Y gyro bias while stationary.
    MPU6050 imu(&device);
    CHECK(imu.Init());
    CHECK(device.registers[0x19] == 9); // 1 kHz / 10 = 100 Hz.
    CHECK(device.registers[0x1a] == 3); // Enables DLPF and the 1 kHz divider base.
    double raw[3], corrected[3];
    CHECK(imu.getGyro(raw));
    CHECK(raw[1] > 1 && raw[1] < 1.01); // getGyro retains deg/s API.
    MPU6050::EulerAngle angle;
    CHECK(imu.getEulerAngleGyro(angle, corrected));
    CHECK(corrected[1] > 0.015 && corrected[1] < 0.02); // radians, same sign.
    for (int i = 0; i < 2000; ++i) CHECK(imu.getEulerAngleGyro(angle, corrected));
    CHECK(std::isfinite(angle.Pitch));
    CHECK(std::abs(corrected[1]) < 0.001); // VQF bias does not become damping torque.
    device.fail = true;
    CHECK(!imu.getEulerAngleGyro(angle, corrected));
    CHECK(!imu.Init());
}
