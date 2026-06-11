#pragma once
#include "EKF.h"
#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>


ExtendedKalmanFilter::ExtendedKalmanFilter(MPU6050 &m): mpu(m){
    setupMotor();
    tankOdomReset();
    mpu = m;
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    if (status != 0) {
        Serial.println(F("MPU6050 init failed"));
        while (1) { delay(100); }
    }
}

void ExtendedKalmanFilter::update(){
    
    int32_t counts[4];
    if (!readMotorEncoderTotals(counts)) {
        Serial.println("failed to read encoder");
        return;
    }

    int32_t curr_left = counts[0];
    int32_t curr_right = counts[1];

    if (!first_read) {
      int32_t delta_counts_left = curr_left - prev_counts_left;
      int32_t delta_counts_right = curr_right - prev_counts_right;

      tankOdomUpdate(delta_counts_left, delta_counts_right);
    } else {
      first_read = false;
    }
    prev_counts_left = curr_left;
    prev_counts_right = curr_right;

    mpu.update();

    float cur_vec[3];
    float prev_vec[3];

    getCurrentDir(cur_vec);
    getLastDir(prev_vec);

    static uint32_t lastMicros = micros();
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    float gyroZ_dps = mpu.getGyroZ();
    float gyroZ_rad_s = gyroZ_dps * DEG2RAD;

    float dYaw_enc = tank_yaw_rad - last_yaw;
    float omega_enc = dYaw_enc / dt;

    ekfYawStep(dt, omega_enc, omega_imu);
}

void ExtendedKalmanFilter::ekfYawStep(float dt, float omega_enc, float omega_imu) {
    float theta_pred = ekf_theta + ekf_omega * dt;
    float omega_pred = omega_enc; 

    float jacobi_F[][2] = {{1.0f, dt},{0, 1.0f.}};

      // Covariance prediction: P = F P F^T + Q
    float covariance_new[][2] = {
        { jacobi_F[0][0] * covariance_new[0][0] + jacobi_F[0][1] * covariance_m[1][0], 
            jacobi_F[0][0] * covariance_m[0][1] + jacobi_F[0][1] * covariance_m[1][1] }, 
        { jacobi_F[1][0] * covariance_m[0][0] + jacobi_F[1][1] * covariance_m[1][0], 
            jacobi_F[0][0] * covariance_m[0][1] + jacobi_F[0][1] * covariance_m[1][1] }
    };

    // Multiply by F^T
    float covariance_pred[][2] =  {
        { jacobi_F[0][0] * covariance_new[0][0] + jacobi_F[0][1] * covariance_new[1][0], 
            jacobi_F[0][0] * covariance_new[0][1] + jacobi_F[0][1] * covariance_new[1][1] }, 
        { jacobi_F[1][0] * covariance_new[0][0] + jacobi_F[1][1] * covariance_m[1][0], 
            jacobi_F[0][0] * covariance_new[0][1] + jacobi_F[0][1] * covariance_new[1][1] }
    };

    covariance_pred[0][0] += Q_theta;
    covariance_pred[1][1] += Q_omega;

    //Innovation: y = z - H x_pred = omega_imu - omega_pred
    float y = omega_imu - omega_pred;

    //Innovation covariance: S = H P_pred H^T + R = P11_pred + R_omega
    float S = covariance_pred[1][1] + R_omega;

    // Kalman gain K = P_pred H^T S^-1 = [P01_pred; P11_pred] / S
    float K0 = covariance_pred[0][1] / S; // for theta
    float K1 = covariance_pred[1][1] / S; // for omega

    ekf_theta = theta_pred + K0 * y;
    ekf_omega = omega_pred + K1 * y;

    float covariance_update[][2] = {
        { (covariance_pred[0][0] - K0) * covariance_pred[1][0],
            (covariance_pred[0][1] - K0) * covariance_pred[1][1]
        },
         { (covariance_pred[1][0] - K1) * covariance_pred[1][0],
            (covariance_pred[1][1] - K1) * covariance_pred[1][1]
        }
    };

    covariance_m = covariance_update;

    serial.println("ekf_theta: ", ekf_theta);
    serial.println("ekf_omega: ", ekf_omega);

}