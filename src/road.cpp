//
//  road.cpp
//  path_planning
//
//  Created by Bryan Bui on 3/16/18.
//

#include "road.hpp"
#include <iostream>
#include "vehicle.hpp"
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(double speed_limit, int lanes) {
  
  this->num_lanes =  lanes;
  this->speed_limit = speed_limit;
  
}

Road::~Road() {}

map<int, Vehicle> Road::get_vehicles() {
  return this->vehicles;
}

map<int, Vehicle>::const_iterator Road::get_vehicle(int id) {
  return this->vehicles.find(id);
}

void Road::clear_traffic() {
  this->vehicles.clear();
  this->vehicles_added = 0;
}

// Function to add cars detected by sensor fusion to our road
void Road::update_traffic(int id, Vehicle car) {
  map<int, Vehicle>::iterator iter = this->vehicles.find(id);
  if(iter == this->vehicles.end()) {
    car.state = "CS";
    this->vehicles_added += 1;
    this->vehicles.insert(std::pair<int,Vehicle>(id,car));
  } else {
    iter->second.a = car.a;
    iter->second.s = car.s;
    iter->second.lane = car.lane;
    iter->second.v = car.v;
  }
}

void Road::add_ego(int lane_num, double s, vector<int> config_data) {
  Vehicle ego = Vehicle(lane_num, s, 0, 0);
  ego.configure(config_data);
  ego.state = "KL";
  this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));
}

void Road::update_ego(int lane_num, double s, double a, double v) {
  this->vehicles.find(this->ego_key)->second.lane = lane_num;
  this->vehicles.find(this->ego_key)->second.s = s;
  this->vehicles.find(this->ego_key)->second.v = v;
  this->vehicles.find(this->ego_key)->second.a = a;
}

Vehicle Road::update() {
  // Generate Predictions
  map<int ,vector<Vehicle> > predictions;
  
  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while(it != this->vehicles.end())
  {
    int v_id = it->first;
    if(v_id != ego_key) {
      vector<Vehicle> preds = it->second.generate_predictions(50);
      predictions[v_id] = preds;
    }
    it++;
  }
  // Generate Trajectory, or return next position for trajectory
  
  it = this->vehicles.find(ego_key);
  vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
  it->second.realize_next_state(trajectory);
  return trajectory[1];
}







