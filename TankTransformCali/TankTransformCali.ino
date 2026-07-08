#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"
#include <MPU6050_light.h>
#include "M5QuadEncoderMotor.h"

#define IMU_I2C_SDA 7
#define IMU_I2C_SCL 6

#define ENC_I2C_SDA 8
#define ENC_I2C_SCL 9

TwoWire I2C_ENCODER = TwoWire(0);
TwoWire I2C_IMU     = TwoWire(1);

M5_4EncoderMotor driver;
int32_t prev_counts_left = 0;
int32_t prev_counts_right = 0;
bool first_read = true;

int count = 0;
void setup() {
  Serial.begin(115200);

  //I2C_IMU.begin(IMU_I2C_SDA, IMU_I2C_SCL, 400000);
  I2C_ENCODER.begin(ENC_I2C_SDA, ENC_I2C_SCL, 400000);
  Wire.beginTransmission(0x26);
  //tk = new TankMotion(mpu);
  if (!driver.begin(I2C_ENCODER)) {
    Serial.println("4EncoderMotor not found");
    while (1) delay(10);
  }

  uint8_t fw = 0;
  if (driver.getFirmwareVersion(fw)) {
    Serial.print("FW: ");
    Serial.println(fw);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    driver.setMode(i, M5_4EncoderMotor::NORMAL_MODE);
  }
  delay(200);

  Serial.println("start");
}

bool moved = false;
unsigned long t0 = 0;

void loop() {
  if (!moved) {
    for (uint8_t i = 0; i < 4; ++i) {
  driver.setMode(i, M5_4EncoderMotor::NORMAL_MODE);
}
    int8_t duty[4] = { -23, -23, 0, 0 };
    driver.setAllDuty(duty);
    t0 = millis();
    moved = true;
    Serial.println("forward for 2s");
  }

  if (moved && millis() - t0 > 2000) {
    int8_t duty[4] = { 0, 0, 0, 0 };
    driver.setAllDuty(duty);
    while (1) delay(10);
  }
}
