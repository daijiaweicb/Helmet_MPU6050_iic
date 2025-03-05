#include "Test.h"

void test()
{
    IIC iic(1);
    MPU mpu;
    MyMPU mypu;
    GetMPU gmu;
    gmu.RegisterSetting(&mypu);

    mpu.beginMPU6050();
    

    iic.iic_close();
}
