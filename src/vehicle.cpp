//
//  vehicle.cpp
//  path_planning
//
//  Created by Bryan Bui on 3/14/18.
//

#include "vehicle.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.hpp"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {
  
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
  
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
  /*
   Here you can implement the transition_function code from the Behavior Planning Pseudocode
   classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
   to the next state.
   
   INPUT: A predictions map. This is a map of vehicle id keys with predicted
   vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
   the vehicle at the current timestep and one timestep in the future.
   OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
   
   */
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;
  cout << this->state << " ln: " << this->lane;
  cout << endl << " :: Cost: "<<endl;
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
      cout << " " << it[0] << " = " << cost << " speed: " << trajectory[1].v << endl << endl;
    }
  }
  
  
  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
  /*
   Provides the possible next states given the current state for the FSM
   discussed in the course, with the exception that lane changes happen
   instantaneously, so LCL and LCR can only transition back to KL.
   */
  
  // TODO: for PLCL/PLCR, check 3rd lane also, so we don't get stuck in either edge.
  vector<string> states;
  string state = this->state;
  int lane = this->lane;
  states.push_back("KL");
  if(state.compare("KL") == 0) {
    if(lane > 0)
      states.push_back("PLCL");
    if(lane < lanes_available - 1)
      states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    states.push_back("PLCL");
    states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
    states.push_back("PLCR");
    states.push_back("LCR");
  } else if (state.compare("LCL") == 0) {
    if(lane > 0)
      states.push_back("LCL");
  } else if (state.compare("LCR") == 0) {
    if(lane < lanes_available - 1)
      states.push_back("LCR");
  }
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
   Given a possible next state, generate the appropriate trajectory to realize the next state.
   */
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
  /*
   Gets next timestep kinematics (position, velocity, acceleration)
   for a given lane. Tries to choose the maximum velocity and acceleration,
   given other vehicle positions and accel/velocity constraints.
   */
  double max_velocity_accel_limit = this->max_acceleration + this->v;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
    } else {
      double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.02 * (this->a);
      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
      
    }
  } else {
    new_velocity = min(max_velocity_accel_limit, this->target_speed);
  }
  
  new_accel = (new_velocity - this->v)*0.02; //Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel * 0.02;
  return{new_position, new_velocity, new_accel};
  
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  /*
   Generate a constant speed trajectory.
   */
  double next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
    Vehicle(this->lane, next_pos, this->v, 0, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
  /*
   Generate a keep lane trajectory.
   */
  vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
  vector<double> kinematics = get_kinematics(predictions, this->lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
  return trajectory;
}


vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
   Generate a trajectory preparing for a lane change.
   */
  double new_s;
  double new_v;
  double new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
  vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
  
  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
    
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    //Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }
  
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
   Generate a lane change trajectory.
   */
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
  vector<double> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
  return trajectory;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
   Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
   rVehicle is updated if a vehicle is found.
   */
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
   Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
   rVehicle is updated if a vehicle is found.
   */
  int min_s = this->s + this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  int temp_id;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    temp_id = it->first;
    if ((temp_vehicle.lane == lane) && (temp_vehicle.s > (this->s + 1)) && (temp_vehicle.s < min_s)) {
//      cout << "\nEGO_V: " << this->v << "Vehicle: " << temp_id << " V: " << temp_vehicle.v << endl;
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

double Vehicle::position_at(int t) {
  double dt = 0.02 * t;
  return this->s + this->v*dt + this->a*dt*dt/2;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  /*
   Generates predictions for non-ego vehicles to be used
   in trajectory generation for the ego vehicle.
   */
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; i++) {
    float next_s = position_at(i);
    float next_v = (position_at(i+1) - s)/0.02;
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }
  return predictions;
  
}

void Vehicle::configure(vector<int> road_data) {
  /*
   Called by simulator before simulation begins. Sets various
   parameters which will impact the ego vehicle.
   */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}
