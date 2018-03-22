//
//  cost.cpp
//  path_planning
//
//  Created by Bryan Bui on 3/16/18.
//

#include "cost.hpp"
#include "vehicle.hpp"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);
const float CHANGING_LANE = pow(10, 4);

///////////
// TODO:
// 1) Add cost for lane changes.



//float on_road_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
//  /*
//   Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
//   Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
//   */
//  float cost = 0;
//  if(vehi)
//  return cost;
//}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
  /*
   Sum weighted cost functions to get total cost for trajectory.
   */
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  float cost = 0.0;
  
  //Add additional cost functions here.
    vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {changing_lane_cost, inefficiency_cost};
    vector<float> weight_list = {CHANGING_LANE, EFFICIENCY};
  
    for (int i = 0; i < cf_list.size(); i++) {
      float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
      cost += new_cost;
    }
  
//  cost = inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
  return cost;
  
}

float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
   Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
   Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
   */
  float cost;
  float distance = 30;
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
//    cost =
  } else {
    cost = 1;
  }
  return cost;
}

float changing_lane_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
   Penalize lane changes.  This helps keep the car in a single lane without swerving too much.
   */
  float cost = 0.0;
  if(data["intended_lane"] == data["final_lane"]) {
    cost = 0;
  } else {
    cost = 0;
  }
  
  return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
  /*
   Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
   */
  
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }
  
  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
  
//  cout << endl <<  " ::: proposed speed: " << proposed_speed_intended;
  
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
  
  return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
  /*
   All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
   we can just find one vehicle in that lane.
   */
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if (vehicle.lane == lane && key != -1) {
      return vehicle.v;
    }
  }
  //Found no vehicle in the lane
  return -1.0;
}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
  /*
   Generate helper data to use in cost functions:
   indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
   final_lane: the lane of the vehicle at the end of the trajectory.
   distance_to_goal: the distance of the vehicle to the goal.
   
   Note that indended_lane and final_lane are both included to help differentiate between planning and executing
   a lane change in the cost functions.
   */
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;
  
  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }
  
  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  return trajectory_data;
}

