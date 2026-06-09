#include <Arduino.h>
#include <Wire.h>
#include "EncoderMotor.h"
#include "TankOdom.h"

int32_t prev_counts_left = 0;
int32_t prev_counts_right = 0;
bool first_read = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(200);
  setupMotor();
  tankOdomReset();
  Serial.println("start");
}
int count = 0;
void loop() {
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