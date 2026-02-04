#ifndef _MPU6050_H_
#define _MPU6050_H_
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68
#define MPU6050_ADDRESS_AD0_HIGH    0x69
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_WHO_AM_I         0x75

class MPU6050 {
public:
    explicit MPU6050(uint8_t address = MPU6050_DEFAULT_ADDRESS) : devAddr(address) {}
    void initialize() {
        Wire.begin();
        I2Cdev::writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, 0x01); // PLL with X axis gyro
        I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, 0x04);  // 200 Hz
        I2Cdev::writeByte(devAddr, MPU6050_RA_CONFIG, 0x03);      // DLPF 42Hz
        I2Cdev::writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, 0x08); // +/-500 deg/s
        I2Cdev::writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, 0x10);// +/-8g
        I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x00);
    }
    bool testConnection() {
        uint8_t who=0; I2Cdev::readBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, &who);
        return (who == 0x68 || who == 0x69);
    }
    void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
        uint8_t buf[14];
        if (I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buf) != 14) { *ax=*ay=*az=*gx=*gy=*gz=0; return; }
        *ax = (int16_t)((buf[0] << 8) | buf[1]);
        *ay = (int16_t)((buf[2] << 8) | buf[3]);
        *az = (int16_t)((buf[4] << 8) | buf[5]);
        *gx = (int16_t)((buf[8] << 8) | buf[9]);
        *gy = (int16_t)((buf[10] << 8) | buf[11]);
        *gz = (int16_t)((buf[12] << 8) | buf[13]);
    }
private:
    uint8_t devAddr;
};

#endif
