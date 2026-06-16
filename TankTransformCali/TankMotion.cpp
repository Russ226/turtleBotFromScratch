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

void TankMotion::forward(float magn_x){
  float cur_dir[3];

  do{
    updateOdom();
    getCurrentDir(cur_dir);

    float err = magn_x - cur_dir[0];
    if (err <= cart_margin_err) break;

    float correction = 5.0f * cur_dir[2];
    int left_m  = (int)(base_speed_f - correction);
    int right_m = (int)(base_speed_f + correction);
    clamp(left_m, right_m);

    int8_t cmd[4] = { (int8_t)left_m, (int8_t)right_m, 0, 0 };
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, cmd, 4);

    delay(5);
    printOdom();
  }while (true);

  int8_t stop_cmd[4] = {0, 0, 0, 0};
  wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, stop_cmd, 4);
}


void TankMotion::reverse(float magn_x){
  float cur_dir[3];

  do {
    updateOdom();
    getCurrentDir(cur_dir);

    float err = cur_dir[0] - magn_x;
    if (err <= cart_margin_err) break;

    float correction = 5.0f * cur_dir[2];
    int left_m  = (int)(base_speed_f - correction);
    int right_m = (int)(base_speed_f + correction);
    clamp(left_m, right_m);

    int8_t cmd[4] = { (int8_t)left_m, (int8_t)right_m, 0, 0 };
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, cmd, 4);

    delay(5);
  }while(true);

  int8_t stop_cmd[4] = {0, 0, 0, 0};
  wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, stop_cmd, 4);
}
void TankMotion::left(float rad){

}
void TankMotion::right(float rad){

}

void TankMotion::updateOdom(){
  int32_t counts[4];
  if (!readMotorEncoderTotals(counts))
    return;

  int32_t curr_left = counts[0];
  int32_t curr_right = counts[1];

  if (!first_read){
    int32_t delta_counts_left = curr_left - prev_counts_left;
    int32_t delta_counts_right = curr_right - prev_counts_right;

    tankOdomUpdate(delta_counts_left, delta_counts_right);
  }
  else{
    first_read = false;
  }

  prev_counts_left = curr_left;
  prev_counts_right = curr_right;
}

void TankMotion::printOdom(){
  float vec[3];
  getCurrentDir(vec);
  Serial.print("x: ");
  Serial.print(vec[0], 3);
  Serial.print("  y: ");
  Serial.print(vec[1], 3);
  Serial.print("  yaw: ");
  Serial.println(vec[2], 3);
}

void TankMotion::calibrateGyroBias() {
  const int N = 500;
  float sum = 0.0f;

  Serial.println("Calibrating gyro bias, keep the tank still...");
  for (int i = 0; i < N; i++) {
    mpu.update();
    float gyroZ_dps   = mpu.getGyroZ();
    float gyroZ_rad_s = gyroZ_dps * deg2rad;
    sum += gyroZ_rad_s;
    delay(5);
  }
  gyro_bias_z = sum / N;

  Serial.print("gyro_bias_z = ");
  Serial.println(gyro_bias_z, 6);
}

void TankMotion::clamp(int &l, int &r){
    if (l < max_speed_f) l = max_speed_f;
    if (l > max_speed_r) l = max_speed_r;
    if (r < max_speed_f) r = max_speed_f;
    if (r > max_speed_r) r = max_speed_r;
}