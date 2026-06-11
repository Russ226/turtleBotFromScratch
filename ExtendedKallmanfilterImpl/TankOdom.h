#pragma once

#include <Arduino.h>


const float meters_per_count_left  = 0.0001137800f;
const float meters_per_count_right = 0.0001052900f;
const int LEFT_SIGN  = -1;
const int RIGHT_SIGN = -1;
const float TRACK_BASELINE_M = 0.111125f; 
const float YAW_SCALE = 0.615f;

void tankOdomReset(float x = 0.0f, float y = 0.0f, float yaw = 0.0f);

void tankOdomUpdate(int32_t delta_counts_left, int32_t delta_counts_right);
// x, y, pheta
void getCurrentDir(float vec[3]);
void getLastDir(float vec[3]);