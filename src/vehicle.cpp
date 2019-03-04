#include "vehicle.h"
#include "math.h"

Vehicle::Vehicle(int _id, double _x, double _y, double _vx, double _vy, double _s, double _d) {
    id = _id;
    x = _x;
    y = _y;
    vx = _vx;
    vy = _vy;
    s = _s;
    d = _d;

    speed = sqrt(pow(vx,2)  + pow(vy,2));
    double const LANE_WIDTH = 4.0;
    lane = (int)(d/LANE_WIDTH);
}

double Vehicle::posIn(double time){
    double check_speed = speed;
    double next_s = s;
    next_s += (time * check_speed);
    return next_s;
}