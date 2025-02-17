#include "Test.h"

void test()
{
    IIC iic(1);
    iic.iic_open();
    initMPU6050(iic);
    
    // 校准陀螺仪零偏（校准时确保传感器静止）
    AngleData calib = {0};
    calibrateSensors(iic, calib, 1000);
    std::cout << "Calibration done: BiasX=" << calib.gyroBiasX 
              << ", BiasY=" << calib.gyroBiasY 
              << ", BiasZ=" << calib.gyroBiasZ << std::endl;
    
    // 初始化角度数据与 Kalman 滤波器
    AngleData prevAngle = {0, 0, 0};
    KalmanFilter kfRoll, kfPitch;
    initKalmanFilter(kfRoll);
    initKalmanFilter(kfPitch);
    
    auto prevTime = std::chrono::high_resolution_clock::now();
    
    try {
        while (true) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            float dt = std::chrono::duration<float>(currentTime - prevTime).count();
            prevTime = currentTime;
            
            SensorData data = readMPU6050(iic);
            AngleData angle = calculateAngle(data, dt, prevAngle, calib, kfRoll, kfPitch);
            prevAngle = angle;
            
            std::cout << "Roll: " << angle.roll << "°, Pitch: " << angle.pitch 
                      << "°, Yaw: " << angle.yaw << "°" << std::endl;
            
            usleep(10000);  // 延时 10ms
        }
    } catch(const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    iic.iic_close();
}
