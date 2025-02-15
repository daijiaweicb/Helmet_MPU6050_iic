#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "iic.h"

int main() {
    IIC iic(1);  // 使用I2C-1总线（树莓派通常为1）
    iic.iic_open();

    try {
        iic.iic_writeRegister(0x6B, 0x00);  // 唤醒MPU6050
        unsigned who_am_i = iic.iic_readRegister(0x75);  // 读取WHO_AM_I寄存器
        std::cout << "WHO_AM_I value: 0x" << std::hex << who_am_i << std::endl;  // 正常应输出0x68
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    iic.iic_close();
    return 0;
}