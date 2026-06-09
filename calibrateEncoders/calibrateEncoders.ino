#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"

int32_t start_counts[4];
bool started = false;
bool finished = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(200);
    setupMotor();

    // Read starting encoder totals
    if (!readMotorEncoderTotals(start_counts)) {
        Serial.println("Failed to read start counts");
    } else {
        Serial.print("Start L: "); Serial.print(start_counts[0]);
        Serial.print("  Start R: "); Serial.println(start_counts[1]);
    }

    // Start moving
    wireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
    started = true;
    Serial.println("Drive robot exactly 1m then press reset or modify code to stop motors.");
}

void loop() {
    if (!started || finished) {
        delay(200);
        return;
    }

    // You might manually stop, or add a timed stop; for pure calibration,
    // just watch counts while you move and then press reset.
    int32_t counts[4];
    if (readMotorEncoderTotals(counts)) {
        Serial.print("Current L: "); Serial.print(counts[0]);
        Serial.print("  Current R: "); Serial.println(counts[1]);
    }

    delay(200);
}