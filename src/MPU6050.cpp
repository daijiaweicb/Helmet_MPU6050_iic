#include "MPU6050.h"
#include <cmath>

//---------------------- Kalman 函数 ----------------------
void initKalmanFilter(KalmanFilter &kf)
{
    kf.angle = 0.0f;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f; kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f; kf.P[1][1] = 0.0f;
    kf.Q_angle = 0.001f;
    kf.Q_bias = 0.003f;
    kf.R_measure = 0.03f;
}

float kalmanUpdate(KalmanFilter &kf, float newRate, float dt, float measuredAngle)
{
    // 预测步骤：利用陀螺仪积分预测角度
    kf.angle += dt * (newRate - kf.bias);
    
    // 更新误差协方差矩阵
    kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + kf.Q_angle);
    kf.P[0][1] -= dt * kf.P[1][1];
    kf.P[1][0] -= dt * kf.P[1][1];
    kf.P[1][1] += kf.Q_bias * dt;
    
    // 测量更新
    float S = kf.P[0][0] + kf.R_measure;
    float K0 = kf.P[0][0] / S;
    float K1 = kf.P[1][0] / S;
    
    float y = measuredAngle - kf.angle;
    kf.angle += K0 * y;
    kf.bias  += K1 * y;
    
    float P00_temp = kf.P[0][0];
    float P01_temp = kf.P[0][1];
    
    kf.P[0][0] -= K0 * P00_temp;
    kf.P[0][1] -= K0 * P01_temp;
    kf.P[1][0] -= K1 * P00_temp;
    kf.P[1][1] -= K1 * P01_temp;
    
    return kf.angle;
}


//---------------------- MPU6050 操作函数 ----------------------
// 校准陀螺仪：计算零偏（要求传感器处于静止状态）
void calibrateSensors(IIC &iic, AngleData &calib, int samples = 1000)
{
    float gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < samples; i++) {
        uint8_t buffer[14];
        if(!iic.readRegisters(0x3B, buffer, 14)) {
            std::cerr << "Calibration read error" << std::endl;
            continue;
        }
        // MPU6050 寄存器 0x43 开始为陀螺仪数据（跳过温度寄存器）
        int16_t gx_raw = (buffer[8] << 8) | buffer[9];
        int16_t gy_raw = (buffer[10] << 8) | buffer[11];
        int16_t gz_raw = (buffer[12] << 8) | buffer[13];
        const float gyroScale = 250.0f / 32768.0f; // ±250°/s 量程
        gx += gx_raw * gyroScale;
        gy += gy_raw * gyroScale;
        gz += gz_raw * gyroScale;
        usleep(10000);  // 每次采样间隔 10ms
    }
    calib.gyroBiasX = gx / samples;
    calib.gyroBiasY = gy / samples;
    calib.gyroBiasZ = gz / samples;
}

// MPU6050 初始化：唤醒、配置陀螺仪量程、低通滤波器及采样率
void initMPU6050(IIC &iic)
{
    iic.iic_writeRegister(0x6B, 0x00);  // 唤醒
    iic.iic_writeRegister(0x1B, 0x00);  // 陀螺仪 ±250°/s
    iic.iic_writeRegister(0x1A, 0x03);  // 低通滤波器 44Hz
    iic.iic_writeRegister(0x19, 0x00);  // 采样率 1kHz（分频器为 0）
}

// 读取 MPU6050 数据：加速度计和陀螺仪共 14 字节（加速度计6字节、温度2字节、陀螺仪6字节）
SensorData readMPU6050(IIC &iic)
{
    SensorData data;
    uint8_t buffer[14];
    if(!iic.readRegisters(0x3B, buffer, 14)) {
        throw std::runtime_error("Failed to read sensor data");
    }
    // 加速度计数据（假设量程 ±2g，转换因子 1/16384）
    int16_t ax_raw = (buffer[0] << 8) | buffer[1];
    int16_t ay_raw = (buffer[2] << 8) | buffer[3];
    int16_t az_raw = (buffer[4] << 8) | buffer[5];
    const float accelScale = 1.0f / 16384.0f;
    data.accelX = ax_raw * accelScale;
    data.accelY = ay_raw * accelScale;
    data.accelZ = az_raw * accelScale;
    
    // 陀螺仪数据（寄存器 0x43 开始）
    int16_t gx_raw = (buffer[8] << 8) | buffer[9];
    int16_t gy_raw = (buffer[10] << 8) | buffer[11];
    int16_t gz_raw = (buffer[12] << 8) | buffer[13];
    const float gyroScale = 250.0f / 32768.0f;
    data.gyroX = gx_raw * gyroScale;
    data.gyroY = gy_raw * gyroScale;
    data.gyroZ = gz_raw * gyroScale;
    
    return data;
}

// 利用加速度计数据计算 Roll 和 Pitch（单位：度）  
float getAccRoll(float accelY, float accelZ) 
{
    return atan2(accelY, accelZ) * 180.0f / M_PI;
}
float getAccPitch(float accelX, float accelY, float accelZ)
{
    return atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / M_PI;
}

//---------------------- 融合 Kalman 滤波的角度计算 ----------------------
// 使用 Kalman 滤波融合陀螺仪积分与加速度计测量，计算 Roll、Pitch（Yaw 仅简单积分）
AngleData calculateAngle(const SensorData &data, float dt, const AngleData &prev, 
                           const AngleData &calib, KalmanFilter &kfRoll, KalmanFilter &kfPitch)
{
    AngleData angle;
    // 修正陀螺仪数据（去除校准零偏）
    float gyroX = data.gyroX - calib.gyroBiasX;
    float gyroY = data.gyroY - calib.gyroBiasY;
    float gyroZ = data.gyroZ - calib.gyroBiasZ;
    
    // 利用加速度计计算角度
    float accRoll  = getAccRoll(data.accelY, data.accelZ);
    float accPitch = getAccPitch(data.accelX, data.accelY, data.accelZ);
    
    // 使用 Kalman 滤波更新 Roll 与 Pitch
    angle.roll  = kalmanUpdate(kfRoll, gyroX, dt, accRoll);
    angle.pitch = kalmanUpdate(kfPitch, gyroY, dt, accPitch);
    // Yaw 仅用简单积分
    angle.yaw = prev.yaw + gyroZ * dt;
    
    return angle;
}
