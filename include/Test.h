#ifndef TEST_H
#define TEST_H

#include <iostream>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "iic.h"
#include "MPU6050.h"

void test();

#endif