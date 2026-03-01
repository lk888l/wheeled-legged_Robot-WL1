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


#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG  0x6B
#define PWR_MGMT_2_REG  0x6C
#define SMPLRT_DIV_REG  0x19
#define MPU_CFG_REG     0X1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define MPU_INTBP_CFG_REG 0X37
#define TEMP_OUT_H_REG  0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU6050_ADDR 0xD0
#define MPU6050_RA_WHO_AM_I 0x75

#define MPU6050_TIME_OUT 100

class MPU6050 {
private:
    I2C_HandleTypeDef* Hi2c = nullptr;


public:
    MPU6050(I2C_HandleTypeDef* _hi2c);
    bool Init();
    bool getGyro(double& _xgyro, double& _ygyro, double& _zgyro);
    bool getAccel(double& _xacc, double& _yacc, double& _zacc);
    bool getTemperature(float& _temp);
};


#endif //__MPU6050_H
