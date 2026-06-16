#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>

class ExtendedKalmanFilter {
  public :
    ExtendedKalmanFilter(MPU6050 &m);
    void update();
  private :
    MPU6050 mpu;
    int32_t prev_counts_left = 0;
    int32_t prev_counts_right = 0;
    bool first_read = true;
    const float Q_theta = .00001f;  // process noise on theta
    const float Q_omega = .0001f;              // process noise on omega (encoder uncertainty)
    const float R_omega = .00005f;              // measurement noise on IMU omega
    const float deg2rad = 3.14159265f / 180.0f;
    float gyro_bias_z = 0.0f;
    float ekf_theta = 0.0f;
    float ekf_omega = 0.0f;
    float ekf_x = 0.0f;
    float ekf_y = 0.0f;
    float covariance_m[2][2] = { {1.0f, 0.0f}, {0.0f, 1.0f} };

    void ekfYawStep(float dt, float omega_enc, float omega_imu);
    void calibrateGyroBias(); 
};