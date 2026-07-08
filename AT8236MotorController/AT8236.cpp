#include "AT8236.h"

AT8236MotorController::AT8236MotorController(int bin1_p,int bin2_p,int ain1_p,int ain2_p,int voltage_p,int e1a_p,int e1b_p,int e2a_p,int e2b_p)
    : bin1Pin(bin1_p),bin2Pin(bin2_p),ain1Pin(ain1_p),ain2Pin(ain2_p),voltagePin(voltage_p),e1aPin(e1a_p),e1bPin(e1b_p),e2aPin(e2a_p),e2bPin(e2b_p) {}

int AT8236MotorController::constrainPWM(int pwm){
    return constrain(pwm, -255, 255);
}

void AT8236MotorController::begin() {
    pinMode(ain1Pin, OUTPUT);
    pinMode(ain2Pin, OUTPUT);
    pinMode(bin1Pin, OUTPUT);
    pinMode(bin2Pin, OUTPUT);
    pinMode(voltagePin, INPUT);

    ledcAttach(ain1Pin, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(ain2Pin, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(bin1Pin, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(bin2Pin, PWM_FREQ, PWM_RES_BITS);

    ledcWrite(ain1Pin, 0);
    ledcWrite(ain2Pin, 0);
    ledcWrite(bin1Pin, 0);
    ledcWrite(bin2Pin, 0);

    analogReadResolution(12);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encLeft.attachFullQuad(e1aPin, e1bPin);
    encRight.attachFullQuad(e2aPin, e2bPin);

    encLeft.setFilter(1023);
    encRight.setFilter(1023);

    encLeft.clearCount();
    encRight.clearCount();
}

void AT8236MotorController::setMotorA(int pwm) {
    pwm = constrainPWM(pwm);

    if (pwm > 0) {
        ledcWrite(ain1Pin, 255);
        ledcWrite(ain2Pin, 255 - pwm);
    } else if (pwm < 0) {
        ledcWrite(ain2Pin, 255);
        ledcWrite(ain1Pin, 255 + pwm);
    } else {
        ledcWrite(ain1Pin, 0);
        ledcWrite(ain2Pin, 0);
    }
}

void AT8236MotorController::setMotorB(int pwm) {
    pwm = constrainPWM(pwm);

    if (pwm > 0) {
        ledcWrite(bin1Pin, 255);
        ledcWrite(bin2Pin, 255 - pwm);
    } else if (pwm < 0) {
        ledcWrite(bin2Pin, 255);
        ledcWrite(bin1Pin, 255 + pwm);
    } else {
        ledcWrite(bin1Pin, 0);
        ledcWrite(bin2Pin, 0);
    }
}

int64_t AT8236MotorController::readLeftEncoder(){
    return encLeft.getCount();
}

int64_t AT8236MotorController::readRightEncoder(){
    return encRight.getCount();
}

void AT8236MotorController::readEncoders(int64_t &left, int64_t &right){
    left = encLeft.getCount();
    right = encRight.getCount();
}

void AT8236MotorController::clearEncoders() {
    encLeft.clearCount();
    encRight.clearCount();
}

double AT8236MotorController::readVoltage(){
    int raw = analogRead(voltagePin);
    return raw * VOLTAGE_SCALE;
}

void AT8236MotorController::readAndClearEncoders(int32_t &leftDelta, int32_t &rightDelta) {
    int64_t left = encLeft.getCount();
    int64_t right = encRight.getCount();

    encLeft.clearCount();
    encRight.clearCount();

    leftDelta = static_cast<int32_t>(left);
    rightDelta = static_cast<int32_t>(right);
}