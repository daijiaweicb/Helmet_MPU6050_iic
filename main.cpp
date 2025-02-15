#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "iic.h"
#include "MPU6050.h"

int main() 
{
    IIC iic(1); // 使用I2C-1总线
    iic.iic_open();
    initMPU6050(iic);

    AngleData prevAngle = {0, 0};
    const float dt = 0.01; // 10ms采样周期

    try {
        while (true) {
            SensorData data = readMPU6050(iic);
            AngleData angle = calculateAngle(data, dt, prevAngle);

            std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch << "°" << std::endl;
            usleep(dt * 1000000); // 等待10ms
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    iic.iic_close();
    return 0;
}