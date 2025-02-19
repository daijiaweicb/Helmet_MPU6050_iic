#include "Test.h"

void test()
{
    IIC iic(1);
    MPU mpu;
    Kalman kal;
    Kalman::KalmanFilter kfRoll;
    Kalman::KalmanFilter kfPitch;
    MPU::AngleData calib;
    MPU::AngleData angle;
    MPU::AngleData prevAngle;
    MPU::SensorData senda;
    iic.iic_open();
    mpu.initMPU6050(iic);
    
    // Calibrate gyro zero bias
    calib = {0};
    mpu.calibrateSensors(iic, calib, 1000);
    std::cout << "Calibration done: BiasX=" << calib.gyroBiasX 
              << ", BiasY=" << calib.gyroBiasY 
              << ", BiasZ=" << calib.gyroBiasZ << std::endl;
    
    // Initializing Angle Data and Kalman Filters
    prevAngle = {0, 0, 0};
    kal.initKalmanFilter(kfRoll);
    kal.initKalmanFilter(kfPitch);
    
    auto prevTime = std::chrono::high_resolution_clock::now();
    
    try {
        while (true) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(currentTime - prevTime).count();
            prevTime = currentTime;
            
            senda = mpu.readMPU6050(iic);
            angle = kal.calculateAngle(senda, dt, prevAngle, calib, kfRoll, kfPitch);
            prevAngle = angle;
            
            std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch 
                      << "°, Yaw: " << angle.yaw << "°" << std::endl;
            
            usleep(10000);  // Delay 10ms
        }
    } catch(const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    iic.iic_close();
}
