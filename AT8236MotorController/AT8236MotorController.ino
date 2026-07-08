#include "AT8236.h"
#include "TankOdom.h"

AT8236MotorController motors(6, 7, 5, 4, 12, 10, 11, 8, 9);
TankOdom odom;

void setup() {
  Serial.begin(115200);
  motors.begin();
  odom.reset();
  delay(10);
}

int count = 0;
void loop() {
  int32_t dLeft = 0;
  int32_t dRight = 0;
  motors.readAndClearEncoders(dLeft, dRight);
  odom.update(dLeft, dRight);
  Serial.print("x=");
  Serial.print(odom.getX(), 6);
  Serial.print(" y=");
  Serial.print(odom.getY(), 6);
  Serial.print(" yaw=");
  Serial.println(odom.getYaw(), 6);
  if(count < 500){
    motors.setMotorA(250);
    motors.setMotorB(250);
    delay(10);
  }else{
    motors.setMotorA(0);
    motors.setMotorB(0);
    delay(10);
  }
  count++;

}
