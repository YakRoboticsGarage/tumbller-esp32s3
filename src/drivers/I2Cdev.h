#ifndef _I2CDEV_H_
#define _I2CDEV_H_
#define I2CDEV_IMPLEMENTATION       I2CDEV_ARDUINO_WIRE
#define I2CDEV_IMPLEMENTATION_WARNINGS
#define I2CDEV_ARDUINO_WIRE         1
#define I2CDEV_BUILTIN_NBWIRE       2
#define I2CDEV_BUILTIN_FASTWIRE     3
#define I2CDEV_I2CMASTER_LIBRARY    4

#include <Arduino.h>
#include <Wire.h>

#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

class I2Cdev {
public:
    static uint8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT) {
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
        if (Wire.endTransmission(false) != 0) return 0;
        uint32_t t0 = millis();
        Wire.requestFrom((int)devAddr, (int)length, (int)true);
        uint8_t count = 0;
        while (Wire.available() && count < length) {
            data[count++] = Wire.read();
        }
        (void)timeout; (void)t0; // timeout not strictly needed with Wire
        return count;
    }

    static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data) {
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
        for (uint8_t i = 0; i < length; i++) Wire.write(data[i]);
        return Wire.endTransmission() == 0;
    }

    static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
        return writeBytes(devAddr, regAddr, 1, &data);
    }
};

#endif
