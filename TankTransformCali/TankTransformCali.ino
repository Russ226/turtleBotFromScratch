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

int count = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin(9,8);

  I2C_IMU.begin(IMU_I2C_SDA, IMU_I2C_SCL, 400000);
  I2C_ENCODER.begin(ENC_I2C_SDA, ENC_I2C_SCL, 400000);
  delay(200);

  Serial.println("start");
}


void loop() {
  // put your main code here, to run repeatedly:

}
