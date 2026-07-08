#include "TankOdom.h"
#include <math.h>

TankOdom::TankOdom()
    : x_(0.0f),
      y_(0.0f),
      yaw_(0.0f),
      last_x_(0.0f),
      last_y_(0.0f),
      last_yaw_(0.0f),
      total_distance_m_(0.0f) {}

void TankOdom::reset(float x, float y, float yaw) {
    x_ = x;
    y_ = y;
    yaw_ = yaw;

    last_x_ = x;
    last_y_ = y;
    last_yaw_ = yaw;

    total_distance_m_ = 0.0f;
}

float TankOdom::normalizeAngle(float angle) {
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void TankOdom::update(int32_t delta_counts_left, int32_t delta_counts_right) {
    last_x_ = x_;
    last_y_ = y_;
    last_yaw_ = yaw_;

    const float left_dist =
        static_cast<float>(delta_counts_left * LEFT_SIGN) * meters_per_count_left;
    const float right_dist =
        static_cast<float>(delta_counts_right * RIGHT_SIGN) * meters_per_count_right;

    const float ds = 0.5f * (left_dist + right_dist);
    const float dtheta =
        ((right_dist - left_dist) / TRACK_BASELINE_M) * YAW_SCALE;

    const float mid_yaw = yaw_ + 0.5f * dtheta;

    x_ += ds * cosf(mid_yaw);
    y_ += ds * sinf(mid_yaw);
    yaw_ = normalizeAngle(yaw_ + dtheta);

    total_distance_m_ += fabsf(ds);
}

void TankOdom::getCurrentDir(float vec[3]) const {
    vec[0] = x_;
    vec[1] = y_;
    vec[2] = yaw_;
}

void TankOdom::getLastDir(float vec[3]) const {
    vec[0] = last_x_;
    vec[1] = last_y_;
    vec[2] = last_yaw_;
}

float TankOdom::getDistance() const {
    return total_distance_m_;
}

float TankOdom::getX() const {
    return x_;
}

float TankOdom::getY() const {
    return y_;
}

float TankOdom::getYaw() const {
    return yaw_;
}