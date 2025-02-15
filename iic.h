#ifndef _IIC_H
#define _IIC_H

#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <thread>


class IIC {
public:
    IIC(int adapter) : adapter_nr(adapter), file(-1) {}
    void iic_open();
    void iic_close();
    unsigned iic_readRegister(uint8_t reg);
    void iic_writeRegister(uint8_t reg, uint8_t value);

private:
    int adapter_nr;
    int file;
    char filename[20];
};


#endif