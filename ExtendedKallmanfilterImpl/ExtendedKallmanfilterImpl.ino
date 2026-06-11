#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>
#include "EKF.h"

#define IMU_I2C_SDA 7
#define IMU_I2C_SCL 6

#define IMU_I2C_SDA 7
#define IMU_I2C_SCL 6

#define ENC_I2C_SDA 8
#define ENC_I2C_SCL 9

TwoWire I2C_ENCODER = TwoWire(0);
TwoWire I2C_IMU     = TwoWire(1);

MPU6050 mpu(I2C_IMU);

int32_t prev_counts_left = 0;
int32_t prev_counts_right = 0;
bool first_read = true;
ExtendedKalmanFilter *ekf = nullptr;
void setup() {
  Serial.begin(115200);
  Wire.begin(9,8);

  I2C_IMU.begin(IMU_I2C_SDA, IMU_I2C_SCL, 400000);
  I2C_ENCODER.begin(ENC_I2C_SDA, ENC_I2C_SCL, 400000);
  delay(200);

  ekf = new ExtendedKalmanFilter(mpu);
  Serial.println("start");
}
int count = 0;
void caliTest1(){
  if (count < 25) {

    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
    delay(10);


    int32_t counts[4];
    if (!readMotorEncoderTotals(counts)) return;

    int32_t curr_left = counts[0];
    int32_t curr_right = counts[1];

    if (!first_read) {
      int32_t delta_counts_left = curr_left - prev_counts_left;
      int32_t delta_counts_right = curr_right - prev_counts_right;

      tankOdomUpdate(delta_counts_left, delta_counts_right);
    } else {
      first_read = false;
    }

    float vec[3];
    getCurrentDir(vec);
    Serial.print("x: ");
    Serial.print(vec[0], 3);
    Serial.print("  y: ");
    Serial.print(vec[1], 3);
    Serial.print("  yaw: ");
    Serial.println(vec[2], 3);
    prev_counts_left = curr_left;
    prev_counts_right = curr_right;

    delay(50);
    count++;
  }else{
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);

  }

}
  

void caliTest2(){
  if (count < 32) {

    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnRight, 4);
    delay(10);


    int32_t counts[4];
    if (!readMotorEncoderTotals(counts)) return;

    int32_t curr_left = counts[0];
    int32_t curr_right = counts[1];

    if (!first_read) {
      int32_t delta_counts_left = curr_left - prev_counts_left;
      int32_t delta_counts_right = curr_right - prev_counts_right;

      tankOdomUpdate(delta_counts_left, delta_counts_right);
    } else {
      first_read = false;
    }

    float vec[3];
    getCurrentDir(vec);
    Serial.print("x: ");
    Serial.print(vec[0], 3);
    Serial.print("  y: ");
    Serial.print(vec[1], 3);
    Serial.print("  yaw: ");
    Serial.println(vec[2], 3);
    prev_counts_left = curr_left;
    prev_counts_right = curr_right;

    delay(50);
    count++;
  }else{
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);

  }

}

void loop() {
  if (count < 32) {

    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
    delay(10);
    ekf->update();
    delay(50);
    count++;
  }else{
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);

  }

}