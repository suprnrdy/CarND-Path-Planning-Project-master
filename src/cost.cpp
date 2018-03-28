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

const double REACH_GOAL = pow(10, 6);
const double EFFICIENCY = pow(10, 5);
const double CHANGING_LANE = pow(10, 1);


double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
  /*
   Sum weighted cost functions to get total cost for trajectory.
   */
  map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;
  
  cost = inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
  return cost;
  
}

double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
  /*
   Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
   Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
   */
  double cost;
  double distance = 30;
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
//    cost =
  } else {
    cost = 1;
  }
  return cost;
}

double changing_lane_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
  /*
   Penalize lane changes.  This helps keep the car in a single lane without swerving too much.
   */
  double cost = 0.0;
  if(data["intended_lane"] == data["final_lane"]) {
    cost = 0;
  } else {
    cost = 0;
  }
  
  return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
  /*
   Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
   
   We want to penalize the lane speed more if it's closer than if it's further away.
   
   */
  
  vector<double> proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
  if (proposed_speed_intended[0] < 0) {
    proposed_speed_intended[0] = vehicle.target_speed;
  }
  
  vector<double> proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
  if (proposed_speed_final[0] < 0) {
    proposed_speed_final[0] = vehicle.target_speed;
  }
  
  double cost = ((2.0*vehicle.target_speed - proposed_speed_intended[0] - proposed_speed_final[0])/vehicle.target_speed) * 1/(proposed_speed_intended[1]);
  
  cout <<  " ::: " << " " << trajectory[1].state << " Target: " << vehicle.target_speed << " Intended: " << proposed_speed_intended[0] << " final: " << proposed_speed_final[0] << endl;
  
  
  return cost;
}

vector<double> lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
  /*
   Check for speed of closest vehicle ahead of us, set that as lane speed.
   */
  vector<double> lane_speed;
  double distance_front = 99999999;
  double speed = -1;
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle traffic = it->second[0];
    if (traffic.lane == lane && key != -1) {
      if(traffic.s > vehicle.s) {
        double gap = traffic.s - vehicle.s;
        if(gap < distance_front) {
          distance_front = gap;
          speed = traffic.v;
        }
      }
    }
  }
//  cout << " Distance Front: " << distance_front << endl;
  lane_speed.push_back(speed);
  lane_speed.push_back(distance_front);
  return lane_speed;
}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
  /*
   Generate helper data to use in cost functions:
   indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
   final_lane: the lane of the vehicle at the end of the trajectory.
   distance_to_goal: the distance of the vehicle to the goal.
   
   Note that indended_lane and final_lane are both included to help differentiate between planning and executing
   a lane change in the cost functions.
   */
  map<string, double> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  double intended_lane;
  
  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }
  
  double distance_to_goal = vehicle.goal_s - trajectory_last.s;
  double final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  return trajectory_data;
}

