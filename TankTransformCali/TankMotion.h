#pragma once
#include <Arduino.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>
// any motion will be in meters and angles radians
class TankMotion {
    public:
        TankMotion(MPU6050 &m);
        void calibrateGyroBias();
        void forward(float magn_x);
    private:
        MPU6050 mpu;
        const float cart_margin_err = 0.05f;
        const float angle_margin_err = 1.0f;
        const int max_speed_f = -23;
        const int max_speed_r = 23;
        int32_t prev_counts_left = 0;
        int32_t prev_counts_right = 0;
        bool first_read = true;

        void TankMotion::clamp(int &l, int &r);
  
};