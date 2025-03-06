#ifndef MPU6050_H
#define MPU6050_H

#include "iic.h"
#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <gpiod.h>
#include <thread>
#include <vector>
#include "Kalman.h"
#include "Callback.h"

#define Interupt_MPU 6
#define chipNo 0

class CallbackInterface
{
public:
    virtual void SensorCallback(float angle) = 0;
    virtual ~CallbackInterface() = default;
};

class MyMPU : public CallbackInterface
{
public:
    void SensorCallback(float angle) override;
};

class MPU
{
public:
    IIC iic;
    struct SensorData
    {
        float gyroX, gyroY, gyroZ;
        float accelX, accelY, accelZ;
    };

    struct AngleData
    {
        float roll, pitch, yaw;
        float gyroBiasX, gyroBiasY, gyroBiasZ;
    };

    std::thread str;
    void initMPU6050(IIC &iic);
    void beginMPU6050();
    void dataReady();
    SensorData readMPU6050(IIC &iic);
    void calibrateSensors(IIC &iic, AngleData &calib, int samples);
    float getAccRoll(float accelY, float accelZ);
    float getAccPitch(float accelX, float accelY, float accelZ);
    AngleData calculateAngle(const SensorData &data, float dt, const AngleData &prev,
                             const AngleData &calib, Kalman::KalmanFilter &kfRoll, Kalman::KalmanFilter &kfPitch);
    void worker()
    {
        running = true;
        while (running)
        {
            const struct timespec ts = {1, 0};
            int r = gpiod_line_event_wait(pin, &ts);
            if (1 == r)
            {
                struct gpiod_line_event event;
                gpiod_line_event_read(pin, &event);
                dataReady();
            }
            else
            {
                running = false;
            }
        }
    }

    MPU():iic(1)
    {}
    

    ~MPU()
    {
         if (str.joinable()) 
         {
            str.join();
         }
        if (pin)
        {
            gpiod_line_release(pin);
            pin = nullptr;
        }
        if (chipGPIO)
        {
            gpiod_chip_close(chipGPIO);
            chipGPIO = nullptr;
        }
        if (owns_iic && iic_ptr)
        {
            iic_ptr->iic_close();
            delete iic_ptr;
        }
    }

private:
    IIC *iic_ptr = nullptr;
    bool owns_iic = false;

    gpiod_chip *chipGPIO = nullptr;
    gpiod_line *pin = nullptr;
    bool running = false;

    AngleData calib;
    AngleData angle;
    AngleData prevAngle;
    SensorData senda;

    Kalman kal;
    Kalman::KalmanFilter kfRoll;
    Kalman::KalmanFilter kfPitch;
    CallbackInterface *callback = nullptr;
};

class GetMPU : public MPU
{
private:
    std::vector<CallbackInterface *> callback;

public:
    void RegisterSetting(CallbackInterface *cb);
};

#endif
