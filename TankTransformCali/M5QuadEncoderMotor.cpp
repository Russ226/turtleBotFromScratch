#include "M5QuadEncoderMotor.h"

M5_4EncoderMotor::M5_4EncoderMotor(uint8_t addr)
    : _addr(addr), _wire(&Wire) {}

bool M5_4EncoderMotor::begin(TwoWire &w)
{
    _wire = &w;
    _wire->begin();
    return ping();
}

bool M5_4EncoderMotor::ping()
{
    _wire->beginTransmission(_addr);
    return _wire->endTransmission() == 0;
}

bool M5_4EncoderMotor::setAllDuty(const int8_t duty[4])
{
    return writeBytes(0x20, reinterpret_cast<const uint8_t *>(duty), 4);
}

bool M5_4EncoderMotor::setDuty(uint8_t motor, int8_t duty)
{
    if (motor > 3)
        return false;
    return writeInt8(0x20 + motor, duty);
}

bool M5_4EncoderMotor::readEncoders(int32_t counts[4])
{
    uint8_t buf[16];
    if (!readBytes(0x30, buf, sizeof(buf)))
        return false;
    for (int i = 0; i < 4; ++i)
    {
        counts[i] = (int32_t)buf[i * 4 + 0] | ((int32_t)buf[i * 4 + 1] << 8) | ((int32_t)buf[i * 4 + 2] << 16) | ((int32_t)buf[i * 4 + 3] << 24);
    }
    return true;
}

bool M5_4EncoderMotor::setEncoder(uint8_t motor, int32_t value)
{
    if (motor > 3)
        return false;
    uint8_t reg = 0x30 + motor * 4;
    uint8_t buf[4] = {
        (uint8_t)(value & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 24) & 0xFF)};
    return writeBytes(reg, buf, 4);
}

bool M5_4EncoderMotor::readSpeeds(int8_t speed[4])
{
    uint8_t buf[4];
    if (!readBytes(0x40, buf, 4))
        return false;
    for (int i = 0; i < 4; ++i)
        speed[i] = (int8_t)buf[i];
    return true;
}

bool M5_4EncoderMotor::setMode(uint8_t motor, MotorMode mode)
{
    if (motor > 3)
        return false;
    return writeUint8(modeBase(motor) + 0, (uint8_t)mode);
}

bool M5_4EncoderMotor::setPositionPID(uint8_t motor, int8_t p, int8_t i, int8_t d)
{
    if (motor > 3)
        return false;
    uint8_t buf[3] = {(uint8_t)p, (uint8_t)i, (uint8_t)d};
    return writeBytes(modeBase(motor) + 1, buf, 3);
}

bool M5_4EncoderMotor::setPositionPoint(uint8_t motor, int32_t point)
{
    if (motor > 3)
        return false;
    uint8_t reg = modeBase(motor) + 4;
    uint8_t buf[4] = {
        (uint8_t)(point & 0xFF),
        (uint8_t)((point >> 8) & 0xFF),
        (uint8_t)((point >> 16) & 0xFF),
        (uint8_t)((point >> 24) & 0xFF)};
    return writeBytes(reg, buf, 4);
}

bool M5_4EncoderMotor::setPositionMaxSpeed(uint8_t motor, int8_t maxSpeed)
{
    if (motor > 3)
        return false;
    return writeInt8(modeBase(motor) + 8, maxSpeed);
}

bool M5_4EncoderMotor::setSpeedPID(uint8_t motor, int8_t p, int8_t i, int8_t d)
{
    if (motor > 3)
        return false;
    uint8_t buf[3] = {(uint8_t)p, (uint8_t)i, (uint8_t)d};
    return writeBytes(modeBase(motor) + 9, buf, 3);
}

bool M5_4EncoderMotor::setSpeedPoint(uint8_t motor, int8_t speedPoint)
{
    if (motor > 3)
        return false;
    return writeInt8(modeBase(motor) + 12, speedPoint);
}

bool M5_4EncoderMotor::getCurrentX100(int32_t &current_x100)
{
    uint8_t buf[4];
    if (!readBytes(0xC0, buf, 4))
        return false;
    current_x100 = (int32_t)buf[0] | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] << 16) | ((int32_t)buf[3] << 24);
    return true;
}

bool M5_4EncoderMotor::getCurrentA(float &amps)
{
    int32_t x100;
    if (!getCurrentX100(x100))
        return false;
    amps = x100 / 100.0f;
    return true;
}

bool M5_4EncoderMotor::getVin8(uint8_t &adc8)
{
    return readBytes(0xA0, &adc8, 1);
}

bool M5_4EncoderMotor::getVin12(uint16_t &adc12)
{
    uint8_t buf[2];
    if (!readBytes(0xB0, buf, 2))
        return false;
    adc12 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return true;
}

bool M5_4EncoderMotor::getVoltage(float &vin)
{
    uint8_t adc8;
    if (!getVin8(adc8))
        return false;
    vin = adc8 / 255.0f * 3.3f / 0.16f;
    return true;
}

bool M5_4EncoderMotor::setEncoderMode(bool ba)
{
    return writeUint8(0xD0, ba ? 1 : 0);
}

bool M5_4EncoderMotor::enableSoftStartStop(uint8_t mask)
{
    return writeUint8(0xD1, mask);
}

bool M5_4EncoderMotor::getFirmwareVersion(uint8_t &ver)
{
    return readBytes(0xF0, &ver, 1);
}

bool M5_4EncoderMotor::setI2CAddress(uint8_t newAddr)
{
    return writeUint8(0xF1, newAddr);
}

uint8_t M5_4EncoderMotor::modeBase(uint8_t motor)
{
    return 0x50 + motor * 0x10;
}

bool M5_4EncoderMotor::writeUint8(uint8_t reg, uint8_t val)
{
    return writeBytes(reg, &val, 1);
}

bool M5_4EncoderMotor::writeInt8(uint8_t reg, int8_t val)
{
    uint8_t b = (uint8_t)val;
    return writeBytes(reg, &b, 1);
}

bool M5_4EncoderMotor::writeBytes(uint8_t reg, const uint8_t *data, size_t len)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    for (size_t i = 0; i < len; ++i)
        _wire->write(data[i]);
    return _wire->endTransmission() == 0;
}

bool M5_4EncoderMotor::readBytes(uint8_t reg, uint8_t *data, size_t len)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0)
        return false;
    size_t n = _wire->requestFrom((int)_addr, (int)len);
    if (n != len)
        return false;
    for (size_t i = 0; i < len; ++i)
        data[i] = _wire->read();
    return true;
}

void M5_4EncoderMotor::printEncoders() {
  int32_t enc[4];
  if (this->readEncoders(enc)) {
    Serial.print("Enc: ");
    for (int i = 0; i < 4; ++i) {
      Serial.print(enc[i]);
      if (i != 3) Serial.print(", ");
    }
    Serial.println();
  } else {
    Serial.println("Encoder read failed");
  }
}

void M5_4EncoderMotor::printTelemetry() {
  float vin, amps;
  if (this->getVoltage(vin)) {
    Serial.print("VIN: ");
    Serial.print(vin, 2);
    Serial.print(" V  ");
  }
  if (this->getCurrentA(amps)) {
    Serial.print("I: ");
    Serial.print(amps, 2);
    Serial.print(" A");
  }
  Serial.println();
}