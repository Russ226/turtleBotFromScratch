#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>

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

void setup() {
  Serial.begin(115200);
  Wire.begin(9,8);

  I2C_IMU.begin(IMU_I2C_SDA, IMU_I2C_SCL, 400000);
  I2C_ENCODER.begin(ENC_I2C_SDA, ENC_I2C_SCL, 400000);
  delay(200);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  if (status != 0) {
    Serial.println(F("MPU6050 init failed"));
    while (1) { delay(100); }
  }
  setupMotor();
  tankOdomReset();
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
    getVector(vec);
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
    getVector(vec);
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
  caliTest2();
   mpu.update();  // must be called regularly

    // raw gyro Z in deg/s (library uses deg/s)
    float gyroZ_dps = mpu.getGyroZ();
    // convert to rad/s for EKF
    const float DEG2RAD = 3.14159265f / 180.0f;
    float gyroZ_rad_s = gyroZ_dps * DEG2RAD;

    Serial.print(F("gyroZ_dps: "));
    Serial.print(gyroZ_dps);
    Serial.print(F("  gyroZ_rad_s: "));
    Serial.println(gyroZ_rad_s);
    delay(20);
}