#include "iic.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

void IIC::iic_open() {
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Can not open i2c device: " << filename << std::endl;
        exit(1);
    }

    //Set the address of MPU6050
    if (ioctl(file, I2C_SLAVE, 0x68) < 0) {
        std::cerr << "Failed to set I2C slave address" << std::endl;
        exit(1);
    }

    std::cout << "i2c device opened: " << filename << std::endl;
}

void IIC::iic_close() {
    if (file >= 0) {
        close(file);
        file = -1;
        std::cout << "i2c device closed" << std::endl;
    }
}

unsigned IIC::iic_readRegister(uint8_t reg) {
    uint8_t tmp[1];
    tmp[0] = reg;
    if (write(file, tmp, 1) != 1) {
        throw std::runtime_error("Failed to write register address");
    }
    if (read(file, tmp, 1) != 1) {
        throw std::runtime_error("Failed to read register value");
    }
    return (unsigned)tmp[0];
}

void IIC::iic_writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tmp[2] = {reg, value};
    if (write(file, tmp, 2) != 2) {
        throw std::runtime_error("Failed to write to i2c");
    }
}