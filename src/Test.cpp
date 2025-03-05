#include "Test.h"

void test()
{
    
    MPU mpu;
    MyMPU mypu;
    GetMPU gmu;

    
    mpu.beginMPU6050();
    
    gmu.RegisterSetting(&mypu);

    
    // iic.iic_close();
}
