#include "Test.h"

void test()
{
    IIC iic(1);
    MPU mpu;
    MyMPU mypu;
    GetMPU gmu;

    iic.iic_open();
    mpu.beginMPU6050();
    gmu.RegisterSetting(&mypu);

    
    iic.iic_close();
}
