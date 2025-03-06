#include "Test.h"

void test()
{

    MPU mpu;
    MyMPU mypu;
    GetMPU gmu;

    gmu.RegisterSetting(&mypu);
    mpu.beginMPU6050();

    // iic.iic_close();
}
