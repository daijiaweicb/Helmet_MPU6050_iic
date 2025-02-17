#include "MPU6050.h"
#include "iic.h"

void calibrateSensors(IIC &iic, AngleData &params, int samples = 1000) {
    float gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < samples; i++) {
        SensorData data = readMPU6050(iic);
        gx += data.gyroX;
        gy += data.gyroY;
        gz += data.gyroZ;
        usleep(1000);
    }
    params.gyroBiasX = gx / samples;
    params.gyroBiasY = gy / samples;
    params.gyroBiasZ = gz / samples;
}

void initMPU6050(IIC &iic) {
    try {
        // 唤醒MPU6050（PWR_MGMT_1寄存器）
        iic.iic_writeRegister(0x6B, 0x00);
        

        // 配置陀螺仪量程 ±250°/s（GYRO_CONFIG寄存器）
        iic.iic_writeRegister(0x1B, 0x00); // 0x00=±250°/s, 0x08=±500°/s, 0x10=±1000°/s, 0x18=±2000°/s
        
        // 配置低通滤波器（可选）
        // iic.iic_writeRegister(0x1A, 0x03); // 带宽44Hz，延迟4.9ms


        // iic.iic_writeRegister(0x19, 0);
        
    } catch (const std::exception &e) {
        std::cerr << "初始化失败: " << e.what() << std::endl;
        exit(1);
    }
}

SensorData readMPU6050(IIC &iic) {
    uint8_t buffer[6];
    SensorData data;

    // 读取14字节数据（从0x3B开始）
    iic.iic_writeRegister(0x43, 0x00); // 设置起始寄存器地址
    ssize_t bytesRead = read(iic.file, buffer, 6);
    if (bytesRead != 6) {
        throw std::runtime_error("读取传感器数据失败");
    }

    // 陀螺仪原始数据（16位有符号）
    int16_t gx = (buffer[0] << 8) | buffer[1];
    int16_t gy = (buffer[2] << 8) | buffer[3];
    int16_t gz = (buffer[4] << 8) | buffer[5];

    // 转换为物理量（根据量程配置）
    const float gyroScale = 250.0 / 32768.0; // ±250°/s时的灵敏度

    data.gyroX = gx * gyroScale;
    data.gyroY = gy * gyroScale;
    data.gyroZ = gz * gyroScale;

    return data;
}



AngleData calculateAngle(const SensorData &data, float dt, AngleData &prev) {
    AngleData angle;

    // 去除陀螺仪零偏
    float gyroX = data.gyroX - prev.gyroBiasX;
    float gyroY = data.gyroY - prev.gyroBiasY;
    float gyroZ = data.gyroZ - prev.gyroBiasZ;

    // 陀螺仪积分
    float gyroRoll = prev.roll + gyroX * dt;
    float gyroPitch = prev.pitch + gyroY * dt;

    return angle;
}