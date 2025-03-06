#include "Test.h"
#include <memory>

void test()
{
    auto myCallback = std::make_shared<MyMPU>();
    MPU mpu;
    

    mpu.RegisterSetting(myCallback);
    mpu.beginMPU6050();

    // iic.iic_close();
}
