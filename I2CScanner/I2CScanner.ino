#include <Arduino.h>
#include <Wire.h>

#define ENC_I2C_SDA 8
#define ENC_I2C_SCL 9
#define I2C_FREQ    400000

TwoWire I2C_ENCODER = TwoWire(0);

static uint8_t g_addr = 0x00;

bool i2cPing(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

bool writeBytes(TwoWire& bus, uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  for (size_t i = 0; i < len; ++i) bus.write(data[i]);
  return bus.endTransmission() == 0;
}

bool writeU8(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t val) {
  return writeBytes(bus, addr, reg, &val, 1);
}

bool readBytes(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  size_t n = bus.requestFrom((int)addr, (int)len);
  if (n != len) return false;
  for (size_t i = 0; i < len; ++i) data[i] = bus.read();
  return true;
}

bool setAllNormalMode(TwoWire& bus, uint8_t addr) {
  bool ok = true;
  ok &= writeU8(bus, addr, 0x50, 0x00);
  ok &= writeU8(bus, addr, 0x60, 0x00);
  ok &= writeU8(bus, addr, 0x70, 0x00);
  ok &= writeU8(bus, addr, 0x80, 0x00);
  return ok;
}

bool setAllDuty(TwoWire& bus, uint8_t addr, int8_t m1, int8_t m2, int8_t m3, int8_t m4) {
  uint8_t buf[4] = {
    (uint8_t)m1, (uint8_t)m2, (uint8_t)m3, (uint8_t)m4
  };
  return writeBytes(bus, addr, 0x20, buf, 4);
}

bool readFirmware(TwoWire& bus, uint8_t addr, uint8_t& fw) {
  return readBytes(bus, addr, 0xF0, &fw, 1);
}

bool readEncoders(TwoWire& bus, uint8_t addr, int32_t enc[4]) {
  uint8_t buf[16];
  if (!readBytes(bus, addr, 0x30, buf, 16)) return false;
  for (int i = 0; i < 4; ++i) {
    enc[i] =  (int32_t)buf[i * 4 + 0]
            | ((int32_t)buf[i * 4 + 1] << 8)
            | ((int32_t)buf[i * 4 + 2] << 16)
            | ((int32_t)buf[i * 4 + 3] << 24);
  }
  return true;
}

void printEncoders(TwoWire& bus, uint8_t addr) {
  int32_t enc[4];
  if (readEncoders(bus, addr, enc)) {
    Serial.print("Encoders: ");
    Serial.print(enc[0]); Serial.print(", ");
    Serial.print(enc[1]); Serial.print(", ");
    Serial.print(enc[2]); Serial.print(", ");
    Serial.println(enc[3]);
  } else {
    Serial.println("Encoder read failed");
  }
}

uint8_t detectModuleAddress(TwoWire& bus) {
  const uint8_t candidates[] = {0x24, 0x26};

  Serial.println("Probing candidate addresses...");
  for (uint8_t addr : candidates) {
    bool ack = i2cPing(bus, addr);
    Serial.print("  0x");
    if (addr < 16) Serial.print('0');
    Serial.print(addr, HEX);
    Serial.print(" -> ");
    Serial.println(ack ? "ACK" : "no ACK");
    if (ack) return addr;
  }
  return 0x00;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("4EncoderMotor direct test");

  I2C_ENCODER.begin(ENC_I2C_SDA, ENC_I2C_SCL, I2C_FREQ);
  delay(100);

  g_addr = detectModuleAddress(I2C_ENCODER);
  if (g_addr == 0x00) {
    Serial.println("No candidate module found at 0x24 or 0x26");
    while (1) delay(10);
  }

  Serial.print("Using address: 0x");
  if (g_addr < 16) Serial.print('0');
  Serial.println(g_addr, HEX);

  uint8_t fw = 0;
  bool fw_ok = readFirmware(I2C_ENCODER, g_addr, fw);
  Serial.print("FW read: ");
  Serial.print(fw_ok ? "OK, value=" : "FAILED, value=");
  Serial.println(fw);

  bool mode_ok = setAllNormalMode(I2C_ENCODER, g_addr);
  Serial.print("Set NORMAL mode on all motors: ");
  Serial.println(mode_ok ? "OK" : "FAILED");

  delay(100);

  Serial.println("Commanding forward on M1/M2 for 2 seconds...");
  bool duty_ok = setAllDuty(I2C_ENCODER, g_addr, -80, -80, 0, 0);
  Serial.print("setAllDuty forward: ");
  Serial.println(duty_ok ? "OK" : "FAILED");

  printEncoders(I2C_ENCODER, g_addr);

  delay(2000);

  Serial.println("Stopping...");
  bool stop_ok = setAllDuty(I2C_ENCODER, g_addr, 0, 0, 0, 0);
  Serial.print("setAllDuty stop: ");
  Serial.println(stop_ok ? "OK" : "FAILED");

  printEncoders(I2C_ENCODER, g_addr);
  Serial.println("Test complete.");
}

void loop() {
}