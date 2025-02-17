#ifndef IIC_H
#define IIC_H

#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdint>
#include <unistd.h>

class IIC {
public:
    IIC(int adapter) : adapter_nr(adapter), file(-1) {}
    void iic_open();
    void iic_close();
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    void iic_writeRegister(uint8_t reg, uint8_t value);
    int file;

private:
    int adapter_nr;
    char filename[20];
};


#endif