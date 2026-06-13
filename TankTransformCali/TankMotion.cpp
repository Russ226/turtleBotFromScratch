#include "TankMotion.h"
#include <Arduino.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>

TankMotion::TankMotion(MPU6050 &m): mpu(m){
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

    calibrateGyroBias();
}

void TankMotion::calibrateGyroBias() {
  const int N = 500;
  float sum = 0.0f;

  Serial.println("Calibrating gyro bias, keep the tank still...");
  for (int i = 0; i < N; i++) {
    mpu.update();
    float gyroZ_dps   = mpu.getGyroZ();
    float gyroZ_rad_s = gyroZ_dps * DEG2RAD;
    sum += gyroZ_rad_s;
    delay(5);
  }
  gyro_bias_z = sum / N;

  Serial.print("gyro_bias_z = ");
  Serial.println(gyro_bias_z, 6);
}

void TankMotion::clamp(int &l, int &r){
    if (left_cmd < max_speed_f) left_cmd = max_speed_f;
    if (left_cmd >  max_speed_r) left_cmd = max_speed_r;
    if (right_cmd < max_speed_f) right_cmd = max_speed_f;
    if (right_cmd >  max_speed_r) right_cmd = max_speed_r;
}