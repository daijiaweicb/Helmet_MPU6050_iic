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



//---------------------- 数据结构 ----------------------
// 存放传感器原始数据：加速度计和陀螺仪（单位已转换为物理量）
struct SensorData {
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
};

// 存放角度数据，同时用来保存校准得到的陀螺仪零偏
struct AngleData {
    float roll, pitch, yaw;
    float gyroBiasX, gyroBiasY, gyroBiasZ;
};

// Kalman 滤波器（单轴）数据结构
struct KalmanFilter {
    float angle;      // 估计的角度
    float bias;       // 估计的陀螺仪零偏
    float P[2][2];    // 误差协方差矩阵
    float Q_angle;    // 过程噪声方差（角度）
    float Q_bias;     // 过程噪声方差（零偏）
    float R_measure;  // 观测噪声方差
};

void initKalmanFilter(KalmanFilter &kf) ;
float kalmanUpdate(KalmanFilter &kf, float newRate, float dt, float measuredAngle);
void calibrateSensors(IIC &iic, AngleData &calib, int samples) ;
void initMPU6050(IIC &iic);
SensorData readMPU6050(IIC &iic);
float getAccRoll(float accelY, float accelZ) ;
float getAccPitch(float accelX, float accelY, float accelZ);
AngleData calculateAngle(const SensorData &data, float dt, const AngleData &prev, 
                           const AngleData &calib, KalmanFilter &kfRoll, KalmanFilter &kfPitch);


#endif

