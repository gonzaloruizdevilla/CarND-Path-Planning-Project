#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "action.h"

using std::vector;

class PathPlanner {
  public:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    vector<double> prev_x_vals;
    vector<double> prev_y_vals;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<Vehicle> vehicles;

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    double end_path_s;
    double end_path_d;

    vector<double> previous_path_x;
    vector<double> previous_path_y;

    vector<Vehicle> sensed_vehicles;

    double ref_vel = 0.0;

    PathPlanner(vector<double> map_waypoints_x,
              vector<double> map_waypoints_y,
              vector<double> map_waypoints_s,
              vector<double> map_waypoints_dx,
              vector<double> map_waypoints_dy);

    void newTelemetry(double car_x,
                      double car_y,
                      double car_s,
                      double car_d,
                      double car_yaw,
                      double car_speed,

                      double end_path_s,
                      double end_path_d,
                      vector<double> previous_path_x,
                      vector<double> previous_path_y,

                      vector<Vehicle> sensed_vehicles);

  private:
    Action getBestAction();
    void planPath(Action action);
};

#endif //PATH_PLANNER_H