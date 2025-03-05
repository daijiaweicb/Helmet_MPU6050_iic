cmake_minimum_required(VERSION 3.10)
project(IIC_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find PkgConfig and libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# Include directories for your own code and gpiod
include_directories(${PROJECT_SOURCE_DIR}/include ${GPIOD_INCLUDE_DIRS})

# Source files
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# Add executable
add_executable(iic_test ${SOURCES})

# Link libraries
target_link_libraries(iic_test PRIVATE i2c ${GPIOD_LIBRARIES})
