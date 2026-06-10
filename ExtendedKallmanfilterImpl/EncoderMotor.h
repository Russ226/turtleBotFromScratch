#pragma once

#include <Arduino.h>
#include <Wire.h>

#define TRACK_LENGTH_M 0.26035f
#define TRACK_HEIGHT_M 0.0762f
#define WHEELBASE_DISTANCE_M 0.111125f
#define DRIVE_DIAMETER_M 0.0508f

#define I2C_ADDR 0x34

#define ADC_BAT_ADDR 0
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_FIXED_PWM_ADDR 31
#define MOTOR_FIXED_SPEED_ADDR 51
#define MOTOR_ENCODER_TOTAL_ADDR 60

#define MOTOR_TYPE_WITHOUT_ENCODER 0
#define MOTOR_TYPE_TT 1
#define MOTOR_TYPE_N20 2
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

extern uint8_t MotorType;
extern uint8_t MotorEncoderPolarity;

extern int8_t car_forward[4];
extern int8_t car_retreat[4];
extern int8_t car_turnRight[4];
extern int8_t car_stop[4];

bool wireWriteByte(uint8_t val);
bool wireWriteDataArray(uint8_t reg, const int8_t *val, unsigned int len);
bool wireReadDataArray(uint8_t reg, uint8_t *buf, size_t len);
bool readMotorEncoderTotals(int32_t counts[4]);
void printEncoderTotals();
void setupMotor();