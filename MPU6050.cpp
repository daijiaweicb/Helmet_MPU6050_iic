#include "MPU6050.h"
#include "iic.h"




void initMPU6050(IIC &iic) {
    try {
        // 唤醒MPU6050（PWR_MGMT_1寄存器）
        iic.iic_writeRegister(0x6B, 0x00);
        
        // 配置加速度计量程 ±2g（ACCEL_CONFIG寄存器）
        iic.iic_writeRegister(0x1C, 0x00); // 0x00=±2g, 0x08=±4g, 0x10=±8g, 0x18=±16g
        
        // 配置陀螺仪量程 ±250°/s（GYRO_CONFIG寄存器）
        iic.iic_writeRegister(0x1B, 0x00); // 0x00=±250°/s, 0x08=±500°/s, 0x10=±1000°/s, 0x18=±2000°/s
        
        // 配置低通滤波器（可选）
        iic.iic_writeRegister(0x1A, 0x06); // 带宽44Hz，延迟4.9ms
    } catch (const std::exception &e) {
        std::cerr << "初始化失败: " << e.what() << std::endl;
        exit(1);
    }
}

SensorData readMPU6050(IIC &iic) {
    uint8_t buffer[14];
    SensorData data;

    // 读取14字节数据（从0x3B开始）
    iic.iic_writeRegister(0x3B, 0x00); // 设置起始寄存器地址
    ssize_t bytesRead = read(iic.file, buffer, 14);
    if (bytesRead != 14) {
        throw std::runtime_error("读取传感器数据失败");
    }

    // 加速度计原始数据（16位有符号）
    int16_t ax = (buffer[0] << 8) | buffer[1];
    int16_t ay = (buffer[2] << 8) | buffer[3];
    int16_t az = (buffer[4] << 8) | buffer[5];

    // 温度数据（可选）
    int16_t temp = (buffer[6] << 8) | buffer[7];

    // 陀螺仪原始数据（16位有符号）
    int16_t gx = (buffer[8] << 8) | buffer[9];
    int16_t gy = (buffer[10] << 8) | buffer[11];
    int16_t gz = (buffer[12] << 8) | buffer[13];

    // 转换为物理量（根据量程配置）
    const float accelScale = 2.0 / 32768.0;  // ±2g时的灵敏度
    const float gyroScale = 250.0 / 32768.0; // ±250°/s时的灵敏度

    data.accelX = ax * accelScale;
    data.accelY = ay * accelScale;
    data.accelZ = az * accelScale;

    data.gyroX = gx * gyroScale;
    data.gyroY = gy * gyroScale;
    data.gyroZ = gz * gyroScale;

    return data;
}



AngleData calculateAngle(const SensorData &data, float dt, AngleData &prev) {
    AngleData angle;

    // 通过加速度计算俯仰角和横滚角
    float accelRoll = atan2(data.accelY, data.accelZ) * 180.0 / M_PI;
    float accelPitch = atan2(-data.accelX, sqrt(data.accelY * data.accelY + data.accelZ * data.accelZ)) * 180.0 / M_PI;

    // 通过陀螺仪积分计算角度
    float gyroRoll = prev.roll + data.gyroX * dt;
    float gyroPitch = prev.pitch + data.gyroY * dt;

    // 互补滤波融合
    angle.roll = 0.98 * gyroRoll + 0.02 * accelRoll;
    angle.pitch = 0.98 * gyroPitch + 0.02 * accelPitch;

    prev = angle;
    return angle;
}