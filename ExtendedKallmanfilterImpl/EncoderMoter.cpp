#include "EncoderMotor.h"

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;

int8_t car_forward[4] = { -23, -23, 0, 0 };
int8_t car_retreat[4] = { 23, 23, 0, 0 };
int8_t car_turnRight[4] = { 0, -23, 0, 0 };
int8_t car_turnLeft[4] = { -23, 0, 0, 0 };
int8_t car_stop[4] = { 0, 0, 0, 0 };

bool wireWriteByte(uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool wireWriteDataArray(uint8_t reg, const int8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) {
    Wire.write((uint8_t)val[i]);
  }
  return Wire.endTransmission() == 0;
}

bool wireReadDataArray(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t n = Wire.requestFrom((int)I2C_ADDR, (int)len);
  if (n != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool readMotorEncoderTotals(int32_t counts[4]) {
  uint8_t buf[16];

  if (!wireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, buf, 16)) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    counts[i] =
      ((int32_t)buf[i * 4 + 0]) | ((int32_t)buf[i * 4 + 1] << 8) | ((int32_t)buf[i * 4 + 2] << 16) | ((int32_t)buf[i * 4 + 3] << 24);
  }

  return true;
}

void printEncoderTotals() {
  int32_t counts[4];
  if (readMotorEncoderTotals(counts)) {
    Serial.print("M1: ");
    Serial.print(counts[0]);
    Serial.print("  M2: ");
    Serial.print(counts[1]);
    Serial.print("  M3: ");
    Serial.print(counts[2]);
    Serial.print("  M4: ");
    Serial.println(counts[3]);
  } else {
    Serial.println("Encoder read failed");
  }
}

void setupMotor() {
  wireWriteDataArray(MOTOR_TYPE_ADDR, (const int8_t *)&MotorType, 1);
  delay(5);
  wireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, (const int8_t *)&MotorEncoderPolarity, 1);
}