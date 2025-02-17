#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "iic.h"
#include "MPU6050.h"

int main() 
{
    IIC iic(1);
    iic.iic_open();
    initMPU6050(iic);

    // 校准零偏，保存到 params 中
    AngleData calib;
    calibrateSensors(iic, calib, 1000); // 校准零偏

    auto prevTime = std::chrono::high_resolution_clock::now();
    // 初始化积分角度为0
    AngleData prevAngle = {0, 0, 0, 0, 0}; // roll, pitch, yaw以及零偏（零偏后面不再使用）

    try {
        while (true) {
            // 计算采样间隔 dt
            auto currentTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(currentTime - prevTime).count();
            prevTime = currentTime;

            SensorData data = readMPU6050(iic);

            // 使用校准零偏进行角度积分
            AngleData angle = calculateAngle(data, dt, prevAngle, calib);
            prevAngle = angle;  // 更新积分结果

            // std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch 
            //           << "°" << std::endl;

            std::cout<< "X: "<< angle.filteredAccelX <<",Y: "<<angle.filteredAccelY<<",Z: "<<angle.filteredAccelZ << std::endl;
        }
        

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    iic.iic_close();
    return 0;
}
