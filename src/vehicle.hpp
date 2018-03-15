//
//  vehicle.hpp
//  path_planning
//
//  Created by Bryan Bui on 3/14/18.
//

#ifndef vehicle_hpp
#define vehicle_hpp

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "json.hpp"

using namespace std;
// for convenience
using json = nlohmann::json;

class Vehicle {
public:
  
  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};
  
  struct collider{
    
    bool collision ; // is there a collision?
    int  time; // time collision happens
    
  };
  
  // Main car's localization Data
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed; //speed
  double acc; //acceleration
  int lane = 1;

  // Previous path data given to the Planner
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
  // Goal
  int goal_lane;
  int goal_s;
  
  // Vehicle Settings
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  float target_speed;
  int lanes_available;
  float max_acceleration;
  
  string state;
  
  /**
   * Constructor
   */
  Vehicle();
//  Vehicle(int lane, float s, float v, float a, string state="CS");
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
//  void update(double x, double y, double s, double d, double yaw, double speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
  void update(json j);
  
  /*
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
  
  vector<string> successor_states();
  
  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
  
  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
  
  vector<Vehicle> constant_speed_trajectory();
  
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
  
  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
  
  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
  
  void increment(int dt);
  
  float position_at(int t);
  
  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  vector<Vehicle> generate_predictions(int horizon=2);
  
  void realize_next_state(vector<Vehicle> trajectory);
  
  void configure(vector<int> road_data);
  */
  
  
  
};


#endif /* vehicle_hpp */
