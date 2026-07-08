#include "TankMotion.h"


TankMotion::TankMotion(TankOdom o, TankMotion m): _odom(o), _motors(m){}


TankMotion::begin(){
    motors.begin();
    odom.reset();
}