#include <Wire.h>
#include <MPU6050_light.h>

#define I2C_SDA 7
#define I2C_SCL 6

MPU6050 mpu(Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  if (status != 0) {
    Serial.println(F("MPU6050 init failed"));
    while (1) { delay(100); }
  }

  Serial.println(F("MPU6050 ready, calibrating..."));
  delay(1000);
  mpu.calcOffsets();  // gyro & accel offsets
  Serial.println(F("Calibration done!"));
}

void loop() {
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