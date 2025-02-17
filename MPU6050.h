#ifndef _MPU6050_H
#define _MPU6050_H

#include "iic.h"
#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <thread>

void initMPU6050(IIC &iic);

struct SensorData {
    float accelX, accelY, accelZ; // 单位：g
    float gyroX, gyroY, gyroZ;    // 单位：°/s
};

struct AngleData {
    float roll, pitch;
    float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
    float gyroX, gyroY, gyroZ;
    float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
};

void calibrateSensors(IIC &iic, AngleData &params, int samples);
SensorData readMPU6050(IIC &iic);
AngleData calculateAngle(const SensorData &data, float dt, const AngleData &prev, const AngleData &calib);


#endif

