/********************************************************************************
  * @file           : MPU6050.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/


#include "MPU6050.h"

MPU6050::MPU6050(I2C_HandleTypeDef *_hi2c)
    : Hi2c(_hi2c)
{

}

bool MPU6050::Init() {
    uint8_t check{};
    uint8_t Data{};

    HAL_I2C_Mem_Read(Hi2c,MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &check, 1, MPU6050_TIME_OUT);
    if(check == 0x68)
    {

        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0x01;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIME_OUT);
        Data = 0x00;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        //  if(rate>1000)rate=1000;
        //  if(rate<4)rate=4;
        //  data=1000/rate-1;
        //  MPU_CFG_REG
        /*
        * <pre>
        *          |   ACCELEROMETER    |           GYROSCOPE
        * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
        * ---------+-----------+--------+-----------+--------+-------------
        * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
        * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
        * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
        * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
        * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
        * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
        * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
        * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
        * </pre>
        */
        Data = 0x09;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_TIME_OUT);
//        Data = 0x02;
//        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, MPU_CFG_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        /**
         * +------------------------+--------------------------+
         * | AFS_SEL       | Full Version |
         * |               |              |
         * +===============+==============+
         * |      0        |      2g      |
         * |      1        |      4g      |
         * |      2        |      8g      |
         * |      3        |      16g     |
         * +------------------------+--------------------------+
         */
        Data = 0x01<<3;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 ->  250 /s
        /**
         * +------------------------+--------------------------+
         * | AFS_SEL       | Full Version |
         * |               |              |
         * +===============+==============+
         * |      0        |  ± 250 °/s   |
         * |      1        |  ± 500 °/s   |
         * |      2        |  ± 1000 °/s  |
         * |      3        |  ± 2000 °/s  |
         * +------------------------+--------------------------+
         */
        Data = 0x02<<3;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        //set MPU INT PIN Config
        Data = 0x80;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, MPU_INTBP_CFG_REG, 1, &Data, 1, MPU6050_TIME_OUT);
        return true;
    }
    return false;
}

bool MPU6050::getGyro(double& _xgyro, double& _ygyro, double& _zgyro)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_StatusTypeDef Hal_bool{};
    Hal_bool = HAL_I2C_Mem_Read(Hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_TIME_OUT);
    if(Hal_bool != HAL_OK){
        return false;
    }
    int16_t Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    int16_t Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    int16_t Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 32768 / (1000)
    _xgyro = Gyro_X_RAW / 30.768;
    _ygyro = Gyro_Y_RAW / 30.768;
    _zgyro = Gyro_Z_RAW / 30.768;
    return true;
}

bool MPU6050::getAccel(double & _xacc, double& _yacc, double& _zacc)
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
    _xacc = Accel_X_RAW / 8192.0;
    _yacc = Accel_Y_RAW / 8192.0;
    _zacc = Accel_Z_RAW / 8192.0;
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
