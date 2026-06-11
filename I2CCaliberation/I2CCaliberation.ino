#include <Wire.h>
#include <Arduino.h>

TwoWire I2C_ENCODER = TwoWire(0);
TwoWire I2C_IMU     = TwoWire(1);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Encoder on GPIO 8 (SDA), 9 (SCL)
  I2C_ENCODER.begin(8, 9, 400000);

  // IMU on GPIO 6 (SDA), 7 (SCL)
  I2C_IMU.begin(6, 7, 400000);
}

void scanBus(TwoWire &bus, const char *name) {
  Serial.print("Scanning "); Serial.println(name);
  byte error, address; int nDevices = 0;
  for (address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    error = bus.endTransmission();
    if (error == 0) {
      Serial.print("Found on "); Serial.print(name);
      Serial.print(" at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (!nDevices) Serial.print("No devices on "), Serial.println(name);
}

void loop() {
  scanBus(I2C_ENCODER, "ENCODER");
  scanBus(I2C_IMU, "IMU");
  delay(2000);
}