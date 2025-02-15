#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "iic.h"

int main()
{
    IIC iic(1);
    iic.iic_open();
    iic.iic_writeRegister(0x25, 0x11);
    unsigned value =iic.iic_readRegister(0x25);
    std:: cout << "value is :" << value << std::endl;
    iic.iic_close();
    return 0;
}