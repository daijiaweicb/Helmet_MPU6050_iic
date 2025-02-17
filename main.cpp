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

    AngleData params;
    calibrateSensors(iic, params,1000); // 校准零偏

    auto prevTime = std::chrono::high_resolution_clock::now();
    AngleData prevAngle = {0, 0};

    try {
        while (true) {
            // 动态计算dt
            auto currentTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(currentTime - prevTime).count();
            prevTime = currentTime;

            SensorData data = readMPU6050(iic);
            AngleData angle = calculateAngle(data, dt, prevAngle);
            prevAngle = angle;

            // std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch << "°" << std::endl;
            std::cout <<"X: " << data.gyroX << ",Y: " << data.gyroY<< ",Z: " << data.gyroZ<<std::endl;

        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    iic.iic_close();
    return 0;
}