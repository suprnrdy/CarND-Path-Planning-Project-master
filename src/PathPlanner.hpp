//
//  PathPlanner.hpp
//  path_planning
//
//  Created by Bryan Bui on 3/19/18.
//

#ifndef PathPlanner_hpp
#define PathPlanner_hpp

#include <stdio.h>

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
//#include "BP.hpp"

using namespace std;

class PP {
private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  
}

#endif /* PathPlanner_hpp */
