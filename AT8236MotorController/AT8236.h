#pragma once
#include "AT8236.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

class AT8236MotorController {
public:
    AT8236MotorController(int bin1_p,int bin2_p,int ain1_p,int ain2_p,int voltage_p,int e1a_p,int e1b_p,int e2a_p,int e2b_p);

    void begin();

    void setMotorA(int pwm);
    void setMotorB(int pwm);

    int64_t readLeftEncoder();
    int64_t readRightEncoder();
    void readEncoders(int64_t &left, int64_t &right);
    void readAndClearEncoders(int32_t &leftDelta, int32_t &rightDelta);
    void clearEncoders();
    double readVoltage();

private:
    int constrainPWM(int pwm);

    const int bin1Pin;
    const int bin2Pin;
    const int ain1Pin;
    const int ain2Pin;
    const int voltagePin;
    const int e1aPin;
    const int e1bPin;
    const int e2aPin;
    const int e2bPin;

    const int PWM_FREQ = 20000;
    const int PWM_RES_BITS = 8;
    const float VOLTAGE_SCALE = 0.05371f;

    ESP32Encoder encLeft;
    ESP32Encoder encRight;
};