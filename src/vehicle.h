#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
  public:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
    int lane;

    Vehicle(int _id,
            double _x,
            double _y,
            double _vx,
            double _vy,
            double _s,
            double _d);

    double posIn(double time);
};

#endif //VEHICLE_H