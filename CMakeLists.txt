cmake_minimum_required(VERSION 3.10)
project(IIC_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

add_executable(iic_test main.cpp iic.cpp MPU6050.cpp Test.cpp)

# 手动链接 libi2c 库
target_link_libraries(iic_test PRIVATE i2c)

