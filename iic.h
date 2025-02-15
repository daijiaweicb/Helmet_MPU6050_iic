#ifndef _IIC_H
#define _IIC_H

#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>


class IIC {
public:
    int file;
    int adapter_nr;
    char filename[20];

    IIC(int adapter) : file(-1), adapter_nr(adapter) {}

    void iic_close();
    void iic_open();
    unsigned iic_readRegister(uint8_t eg);
    void iic_writeRegister(uint8_t reg , uint8_t value);


    ~IIC() {
        iic_close();
    }
    
    
};


#endif