#include <vector>
#include "path_planner.h"
#include "vehicle.h"
#include "action.h"
#include "helpers.h"
#include <iostream>
#include "spline.h"

#include <math.h>

using std::vector;

double const LANE_WIDTH = 4.0;
double const MAX_SPEED_MPH = 49.5;
double const TICK = 0.02;
double const MPH_MS = 2.24;

double const TRACK_LENGTH = 6945.554;
double const LANES = 3;

double const MAX_ACCELERATION = 5;
double const MAX_JERK = 10;

double const S_LENGTH = 6914.14925765991;

double const SAFETY_DISTANCE = 20;
double const SAFETY_CHANGE_LANE_DISTANCE_FRONT = 10;
double const SAFETY_CHANGE_LANE_DISTANCE_BACK = 10;

PathPlanner::PathPlanner(vector<double> map_waypoints_x,
                         vector<double> map_waypoints_y,
                         vector<double> map_waypoints_s,
                         vector<double> map_waypoints_dx,
                         vector<double> map_waypoints_dy)
{
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
};

void PathPlanner::newTelemetry(double car_x,
                        double car_y,
                        double car_s,
                        double car_d,
                        double car_yaw,
                        double car_speed,

                        double end_path_s,
                        double end_path_d,

                        vector<double> previous_path_x,
                        vector<double> previous_path_y,

                        vector<Vehicle> sensed_vehicles) {

    this->car_x = car_x;
    this->car_y = car_y;
    this->car_s = car_s;
    this->car_d = car_d;
    this->car_yaw = car_yaw;
    this->car_speed = car_speed;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->sensed_vehicles = sensed_vehicles;

    Action action = getBestAction();
    planPath(action);
}

Action PathPlanner::getBestAction() {

    int prev_size = previous_path_x.size();

    
    if (prev_size > 0){
        car_s = end_path_s;
        car_d = end_path_d;
    }

    bool too_close = false;

    
    vector<double> lane_speeds = { MAX_SPEED_MPH,  MAX_SPEED_MPH, MAX_SPEED_MPH };
    vector<double> lane_next_car_s = {10000, 10000, 10000};

    //find top lane speeds based on present vehicles
    double time = (double)prev_size * TICK;
    int lane = (int)(car_d / LANE_WIDTH);
    for (Vehicle vehicle : sensed_vehicles) {
        double next_car_s = vehicle.posIn(time);
        double dist = fmod(next_car_s - car_s + S_LENGTH, S_LENGTH);
        if (lane_speeds[vehicle.lane] > vehicle.speed && dist < 100){ //car is in front
            lane_speeds[vehicle.lane] = vehicle.speed;
        }
    }
    //find max speed lane
    double max_lane_speed = 0;
    double max_lane = 1;
    for(int i = 2; i>=0; i--){
        if (max_lane_speed < lane_speeds[i]){
            max_lane = i;
            max_lane_speed = lane_speeds[i];
        }
    }
    //always prefer center lane, so we can better choose future lane changes
    if (max_lane_speed == lane_speeds[1]){
        max_lane = 1;
    }

    for (Vehicle vehicle : sensed_vehicles)
    {
        if (lane == vehicle.lane)
        {
            double next_car_s = vehicle.posIn(time);
            double dist = fmod(next_car_s - car_s + S_LENGTH, S_LENGTH);
            if (dist < SAFETY_DISTANCE)
            {
                too_close = true;
            }
        }
    }
    int next_lane = -1;
    if (max_lane > lane){
        next_lane = lane + 1;
    } else if (max_lane < lane){
        next_lane = lane - 1;
    }
    //when changing lanes, verify it's safe
    if (next_lane > -1) {
        for (Vehicle vehicle : sensed_vehicles) {
            // next_lane == vehicle.lane  doen't detect vehicles changing lanes
            // so this doesn't avoid collisions
            // either we should track others vehicle position on d axis, or if d is not centered, which is simpler
            double min_d = 4 * next_lane;
            double max_d = 4 * (next_lane +1);
            if (next_lane < 2) max_d += 1; 
            if (next_lane > 0) min_d -= 1;
            if (min_d < vehicle.d && max_d > vehicle.d) {
                double next_car_s = vehicle.posIn(time);
                double dist = fmod(next_car_s - car_s + S_LENGTH, S_LENGTH);
                if (dist < SAFETY_CHANGE_LANE_DISTANCE_FRONT) {
                    next_lane = -1;
                }
                dist = fmod(car_s - next_car_s + S_LENGTH, S_LENGTH);
                if (dist < SAFETY_CHANGE_LANE_DISTANCE_BACK){
                    next_lane = -1;
                }
            }
        }
        if (next_lane > -1) {
            std::cout << "Found safe change to fastest lane: " << next_lane << ". Lane max speed: " << lane_speeds[next_lane] << std::endl;
            std::cout << "Current lane: " << lane << ". Current max speed: " <<lane_speeds[lane] << std::endl;
        }
    }
    lane = next_lane > -1 ? next_lane: lane;

    if (too_close)
    {
        ref_vel -= .224;
    }
    else if (ref_vel < MAX_SPEED_MPH)
    {
        ref_vel += .224;
    }

    Action action;
    action.reference_velocity = ref_vel;
    action.lane = lane;
    return action;
    
}

void PathPlanner::planPath(Action action) {
    double ref_vel = action.reference_velocity;;
    int lane = action.lane;
    int prev_size = previous_path_x.size();
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if(prev_size < 2 ){
        double  prev_car_x = car_x - cos(car_yaw);
        double  prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    

    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++)
    {
        
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        
    }
    
    //spline
    

    tk::spline s;
    s.set_points(ptsx, ptsy);

    next_x_vals.clear();
    next_y_vals.clear();

    for(int i = 0; i < previous_path_x.size(); i++){
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    //calculate how to break up spline to that we travel at desired velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
    
    double x_add_on = 0;

    for (int i = 0; i < 50 - previous_path_x.size(); i++) {
        
        double N = target_dist / (TICK * (ref_vel / MPH_MS));
        double x_point = x_add_on + (target_x/N);
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}
