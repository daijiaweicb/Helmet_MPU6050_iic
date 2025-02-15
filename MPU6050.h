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
    float roll, pitch; // 横滚角、俯仰角（单位：度）
};

SensorData readMPU6050(IIC &iic);
AngleData calculateAngle(const SensorData &data, float dt, AngleData &prev);

#endif

