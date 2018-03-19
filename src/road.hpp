//
//  road.hpp
//  path_planning
//
//  Created by Bryan Bui on 3/16/18.
//

#ifndef road_hpp
#define road_hpp

#include <stdio.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.hpp"

using namespace std;

class Road {
public:
  int ego_key = -1;
  int num_lanes;
  double speed_limit;
  map<int, Vehicle> vehicles;
  int vehicles_added = 0;
  
  /**
   * Constructor
   */
  Road(double speed_limit, int lanes);
  
  /**
   * Destructor
   */
  virtual ~Road();
  
  map<int, Vehicle> get_vehicles();
  
  map<int, Vehicle>::const_iterator get_vehicle(int id);
  void update_traffic(int id, Vehicle car);
  void clear_traffic();
  
  Vehicle update();
  
  void add_ego(int lane_num, double s, vector<int> config_data);
  void update_ego(int lane_num, double s, double a, double v);
  
  void cull();
  
};


#endif /* road_hpp */
