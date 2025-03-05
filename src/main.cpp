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

int main()
{
    test();
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
