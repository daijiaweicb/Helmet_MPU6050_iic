#include <iostream>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "iic.h"
#include "MPU6050.h"
#include "Test.h"

#define MPU6050_ADDR 0x68

//---------------------- 主函数 ----------------------
int main() 
{
    test();    
    return 0;
}
