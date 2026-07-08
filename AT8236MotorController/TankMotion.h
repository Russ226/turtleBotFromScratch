#pragma once
#include "TankMotion.h"
#include "TankOdom.h"
#include "AT8236.h"


class TankMotion{
    public:
        TankMotion(TankOdom o, TankMotion m);
        begin();
    private:
        TankOdom _odom;
        AT8236MotorController _motors;
};