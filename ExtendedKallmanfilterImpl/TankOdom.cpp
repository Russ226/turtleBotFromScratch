#include "TankOdom.h"


float tank_x_m = 0.0f;
float tank_y_m = 0.0f;
float tank_yaw_rad = 0.0f;

void tankOdomReset(float x, float y, float yaw)
{
    tank_x_m = x;
    tank_y_m = y;
    tank_yaw_rad = yaw;
}

void tankOdomUpdate(int32_t delta_counts_left, int32_t delta_counts_right)
{
    // 1. Convert counts → signed distances for each track
    float dL = LEFT_SIGN  * delta_counts_left  * meters_per_count_left;
    float dR = RIGHT_SIGN * delta_counts_right * meters_per_count_right;

    // 2. Track kinematics: arc distance and heading change
    float dS    = 0.5f * (dL + dR);                    // center travel
    float dYaw  = (YAW_SCALE *(dR - dL) )/ TRACK_BASELINE_M;        // heading change
    float yaw_mid = tank_yaw_rad + 0.5f * dYaw;

    // 3. Update pose in world frame
    tank_x_m    += dS * cosf(yaw_mid);
    tank_y_m    += dS * sinf(yaw_mid);
    tank_yaw_rad += dYaw;
}

void getVector(float vec[3]){
  vec[0] = tank_x_m;
  vec[1] = tank_y_m;
  vec[2] = tank_yaw_rad;
}