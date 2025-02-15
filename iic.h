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
    unsigned iic_readRegister(u_int8_t reg);
    void iic_writeRegister(u_int8_t reg , unsigned value);


    ~IIC() {
        iic_close();
    }
    
    
};


#endif