/********************************************************************************
  * @file           : MPU6050.cpp
  * @author         : Luka
  * @brief          : None
  * @attention      : None
  * @date           : 26-2-23
  *******************************************************************************/


#include <utility>
#include <cmath>
#include "MPU6050.h"

MPU6050::MPU6050(I2C_HandleTypeDef *_hi2c)
    : Hi2c(_hi2c)
    ,vqf((1.0/M650_cfg.SampleRate))
{

}

MPU6050::MPU6050(I2C_HandleTypeDef *_hi2c, MPU6050::InitConfig_t _cfg)
    : Hi2c(_hi2c)
    , M650_cfg(_cfg)
    ,vqf((1.0/M650_cfg.SampleRate))
{

}

bool MPU6050::Init() {
    uint8_t check{};
    uint8_t Data{};

    HAL_I2C_Mem_Read(Hi2c,MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_TIME_OUT);
    if(check == 0x68)
    {

        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0x01;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIME_OUT);
        Data = 0x00;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        // Set DATA RATE of 4 - 1000Hz by writing SMPLRT_DIV register
        if(M650_cfg.SampleRate>1000)
        {M650_cfg.SampleRate=1000;}
        else if(M650_cfg.SampleRate<4)
        {M650_cfg.SampleRate=4;}
        Data = 1000/M650_cfg.SampleRate - 1;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_TIME_OUT);

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
//        if (M650_cfg.SampleRate >= 500) {
//            Data = 0x01; // 184Hz Bandwidth
//        } else if (M650_cfg.SampleRate >= 200) {
//            Data = 0x02; // 94Hz Bandwidth
//        } else if (M650_cfg.SampleRate >= 100) {
//            Data = 0x03; // 44Hz Bandwidth
//        } else if (M650_cfg.SampleRate >= 50) {
//            Data = 0x04; // 21Hz Bandwidth
//        } else if (M650_cfg.SampleRate >= 25) {
//            Data = 0x05; // 10Hz Bandwidth
//        } else {
//            Data = 0x06; // 5Hz Bandwidth
//        }
        Data = 0x00;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, MPU_CFG_REG, 1, &Data, 1, MPU6050_TIME_OUT);

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
        Data = (static_cast<uint8_t>(M650_cfg.AccRange))<<3;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_TIME_OUT);
        if(M650_cfg.AccRange == AccRange_t::A2)
        {AccCoefficient = 32768 / 2.0;}
        else if(M650_cfg.AccRange == AccRange_t::A4)
        {AccCoefficient = 32768 / 4.0;}
        else if(M650_cfg.AccRange == AccRange_t::A8)
        {AccCoefficient = 32768 / 8.0;}
        else if(M650_cfg.AccRange == AccRange_t::A16)
        {AccCoefficient = 32768 / 16.0;}
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
        Data = (static_cast<uint8_t>(M650_cfg.GyroRange))<<3;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_TIME_OUT);
        if(M650_cfg.GyroRange == GyroRange_t::G250)
        {GyroCoefficient = 32768 / 250.0;}
        else if(M650_cfg.GyroRange == GyroRange_t::G500)
        { GyroCoefficient = 32768 / 500.0;}
        else if(M650_cfg.GyroRange == GyroRange_t::G1000)
        { GyroCoefficient = 30768 / 1000.0;}
        else if(M650_cfg.GyroRange == GyroRange_t::G2000)
        { GyroCoefficient = 32768 / 2000.0;}
        //set MPU INT PIN Config
        Data = 0x80;
        HAL_I2C_Mem_Write(Hi2c, MPU6050_ADDR, MPU_INTBP_CFG_REG, 1, &Data, 1, MPU6050_TIME_OUT);

        // Init VQF ---- Attitude calculation algorithm
//        vqf = VQF((1.0/M650_cfg.SampleRate));
        return true;
    }
    return false;
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
    _acc[0] = Accel_X_RAW / 8192.0;
    _acc[1] = Accel_Y_RAW / 8192.0;
    _acc[2] = Accel_Z_RAW / 8192.0;
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
    if((getGyro(gyro) && getAccel(acc)))
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
    if((getGyro(gyro) && getAccel(acc)))
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
    if((getGyro(gyro) && getAccel(acc)))
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

