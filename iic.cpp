#include "iic.h"

 void IIC ::iic_open()
 {
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Can not open i2c device: " << filename << std::endl;
        exit(1);
    }
    std::cout << "i2c device has been opened successfully " << filename << std::endl;

 }

 void IIC :: iic_close()
 {
    if (file >= 0) {
            close(file);
            file = -1;
            std::cout << "i2c device has been closed" << std::endl;
        }
 }

unsigned IIC::iic_readRegister(uint8_t reg)
{
    uint8_t tmp[2];
    tmp[0] = reg;
    write(file,&tmp,1);
    long int r = read(file,tmp,2);
    if (r < 0) {
        throw std::runtime_error("Could not write register address");
    }

    return (((unsigned)(tmp[0])) << 8) | ((unsigned)(tmp[1]));
}

void IIC::iic_writeRegister(uint8_t reg, unsigned value)
{
    uint8_t tmp[3];
    tmp[0] = reg;
    tmp[1] = (char)((value & 0xff00) >> 8);
	tmp[2] = (char)(value & 0x00ff);
	long int r = write(file,&tmp,3);

    if (r < 0) {
        throw std::runtime_error("Could not write to i2c");
    }
}

