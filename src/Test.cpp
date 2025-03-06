#include "Test.h"

void test()
{

    MPU mpu;
    MyMPU mypu;

    mpu.RegisterSetting(&mypu);
    mpu.beginMPU6050();

    // iic.iic_close();
}
