#pragma once
#include <Arduino.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>
// any motion will be in meters and angles radians
class TankMotion {
    public:
        TankMotion(MPU6050 &m);
        void forward(float magn_x);
        void reverse(float magn_x);
        void left(float rad);
        void right(float rad);
        void printOdom();
    private:
        MPU6050 mpu;
        const float cart_margin_err = 0.05f;
        const float angle_margin_err = 0.0002f;
        const int max_speed_f = -46;
        const int max_speed_r = 46;
        const int base_speed_f = -23;
        const int base_speed_r = 23;
        const float deg2rad = 3.14159265f / 180.0f;
        float gyro_bias_z = 0.0f;
        int32_t prev_counts_left = 0;
        int32_t prev_counts_right = 0;
        bool first_read = true;
        float correction_amount = 1.2f;  
        
        void updateOdom();
        void calibrateGyroBias();
        void clamp(int &l, int &r);

};