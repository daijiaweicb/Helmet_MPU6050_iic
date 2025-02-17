#include <iostream>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// I2C 设备路径及 MPU6050 地址
#define I2C_DEV "/dev/i2c-1"
#define MPU6050_ADDR 0x68

//---------------------- IIC 类 ----------------------
struct IIC {
    int file;
    const char* devPath;
    int addr;

    IIC(const char* path, int addr): devPath(path), addr(addr), file(-1) {}

    bool openDevice() {
        file = open(devPath, O_RDWR);
        if (file < 0) {
            std::cerr << "Error opening I2C device" << std::endl;
            return false;
        }
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            std::cerr << "Error setting I2C address" << std::endl;
            return false;
        }
        return true;
    }

    void closeDevice() {
        if(file >= 0)
            close(file);
    }

    // 写寄存器
    bool writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = { reg, value };
        if (write(file, buffer, 2) != 2) {
            std::cerr << "Error writing to I2C register 0x" 
                      << std::hex << int(reg) << std::dec << std::endl;
            return false;
        }
        return true;
    }

    // 读多个寄存器
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
        if (write(file, &reg, 1) != 1) {
            std::cerr << "Error writing register address" << std::endl;
            return false;
        }
        if (read(file, buffer, length) != (ssize_t)length) {
            std::cerr << "Error reading registers" << std::endl;
            return false;
        }
        return true;
    }
};

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

//---------------------- Kalman 函数 ----------------------
void initKalmanFilter(KalmanFilter &kf) {
    kf.angle = 0.0f;
    kf.bias = 0.0f;
    kf.P[0][0] = 0.0f; kf.P[0][1] = 0.0f;
    kf.P[1][0] = 0.0f; kf.P[1][1] = 0.0f;
    kf.Q_angle = 0.001f;
    kf.Q_bias = 0.003f;
    kf.R_measure = 0.03f;
}

float kalmanUpdate(KalmanFilter &kf, float newRate, float dt, float measuredAngle) {
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
void calibrateSensors(IIC &iic, AngleData &calib, int samples = 1000) {
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
void initMPU6050(IIC &iic) {
    iic.writeRegister(0x6B, 0x00);  // 唤醒
    iic.writeRegister(0x1B, 0x00);  // 陀螺仪 ±250°/s
    iic.writeRegister(0x1A, 0x03);  // 低通滤波器 44Hz
    iic.writeRegister(0x19, 0x00);  // 采样率 1kHz（分频器为 0）
}

// 读取 MPU6050 数据：加速度计和陀螺仪共 14 字节（加速度计6字节、温度2字节、陀螺仪6字节）
SensorData readMPU6050(IIC &iic) {
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
float getAccRoll(float accelY, float accelZ) {
    return atan2(accelY, accelZ) * 180.0f / M_PI;
}
float getAccPitch(float accelX, float accelY, float accelZ) {
    return atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / M_PI;
}

//---------------------- 融合 Kalman 滤波的角度计算 ----------------------
// 使用 Kalman 滤波融合陀螺仪积分与加速度计测量，计算 Roll、Pitch（Yaw 仅简单积分）
AngleData calculateAngle(const SensorData &data, float dt, const AngleData &prev, 
                           const AngleData &calib, KalmanFilter &kfRoll, KalmanFilter &kfPitch) {
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

//---------------------- 主函数 ----------------------
int main() {
    IIC iic(I2C_DEV, MPU6050_ADDR);
    if(!iic.openDevice())
        return 1;
    
    initMPU6050(iic);
    
    // 校准陀螺仪零偏（校准时确保传感器静止）
    AngleData calib = {0};
    calibrateSensors(iic, calib, 1000);
    std::cout << "Calibration done: BiasX=" << calib.gyroBiasX 
              << ", BiasY=" << calib.gyroBiasY 
              << ", BiasZ=" << calib.gyroBiasZ << std::endl;
    
    // 初始化角度数据与 Kalman 滤波器
    AngleData prevAngle = {0, 0, 0};
    KalmanFilter kfRoll, kfPitch;
    initKalmanFilter(kfRoll);
    initKalmanFilter(kfPitch);
    
    auto prevTime = std::chrono::high_resolution_clock::now();
    
    try {
        while (true) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(currentTime - prevTime).count();
            prevTime = currentTime;
            
            SensorData data = readMPU6050(iic);
            AngleData angle = calculateAngle(data, dt, prevAngle, calib, kfRoll, kfPitch);
            prevAngle = angle;
            
            std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch 
                      << "°, Yaw: " << angle.yaw << "°" << std::endl;
            
            usleep(10000);  // 延时 10ms
        }
    } catch(const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    iic.closeDevice();
    return 0;
}
