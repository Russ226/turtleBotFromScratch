#pragma once
#include "TankOdom.h"
#include <Arduino.h>

class TankOdom {
public:
    TankOdom();

    void reset(float x = 0.0f, float y = 0.0f, float yaw = 0.0f);
    void update(int32_t delta_counts_left, int32_t delta_counts_right);

    void getCurrentDir(float vec[3]) const;
    void getLastDir(float vec[3]) const;

    float getDistance() const;

    float getX() const;
    float getY() const;
    float getYaw() const;

private:
    static constexpr float count_bias = 0.94f;
    static constexpr float meters_per_count_left  = 0.0001137800f * count_bias;
    static constexpr float meters_per_count_right = 0.0001052900f * count_bias;
    static constexpr int LEFT_SIGN  = -1;
    static constexpr int RIGHT_SIGN = -1;
    static constexpr float TRACK_BASELINE_M = 0.111125f / 1.5f;
    static constexpr float YAW_SCALE = 0.615f;

    float x_;
    float y_;
    float yaw_;

    float last_x_;
    float last_y_;
    float last_yaw_;

    float total_distance_m_;

    static float normalizeAngle(float angle);
};