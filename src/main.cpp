#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"
#include "vehicle.hpp"
#include "road.hpp"

using namespace std;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // start in lane 1;
  int lane = 1;
  // Have a reference velocity to target
  double ref_vel = 0; //mpg
  
  // Configure our road
  Road road = Road(SPEED_LIMIT, NUM_LANES);
  vector<int> ego_config = {SPEED_LIMIT,NUM_LANES,30,lane,MAX_ACCEL};
  road.add_ego(lane, 0, ego_config);
  double prev_speed = 0;
  
  h.onMessage([&prev_speed, &road,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto  j= json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
          /////////////////////////////////////////////////
          // SENSOR FUSION DATA
          // Parse JSON data
          // j[1] is the data JSON object
          /////////////////////////////////////////////////
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          
          /////////////////////////////////////////////////
          // Update our Ego on the road
          /////////////////////////////////////////////////
          int car_lane = car_d / 4;
          
          double car_accel = (car_speed - prev_speed)/0.02;
          road.update_ego(car_lane, car_s, car_accel, car_speed);
          Vehicle currentv = road.get_vehicle(road.ego_key)->second;
//          cout << "Current State: " << currentv.state << " lane: " << currentv.lane << " speed: " << currentv.v << " acc: " << currentv.a << endl;
          
          prev_speed = car_speed;
        
          
          /////////////////////////////////////////////////
          // Update the traffic on the road
          // [  0, 1, 2,  3,  4, 5, 6]
          // [ id, x, y, vx, vy, s, d]
          /////////////////////////////////////////////////
          
          map<int, Vehicle> past_vehicles = road.get_vehicles();
          
          int v_lane_count = 0;
          //Go through each vehicle, derive V and A, then update the Road
          road.clear_traffic();
          for(int i = 0; i < sensor_fusion.size(); i++) {
            int vd = sensor_fusion[i][6];
            // If the obstacles are all in the same side of the road as us
            if(vd >= 0) {
              v_lane_count++;
              int vid = sensor_fusion[i][0];
              double vs = sensor_fusion[i][5];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double vspeed = sqrt(vx*vx+vy*vy)*2.23;
              int vlane = vd / 4;
              // check if vehicle ID exists in list.
              // if it does: calculate acceleration and update it.
              // if not set acceleration to zero
              double vaccel = 0;
              map<int, Vehicle>::iterator iter = past_vehicles.find(vid);
              if(iter != past_vehicles.end()) {
                // Vehicle exists, update it.
                // Calculate Acceleration
                vaccel = (vspeed - iter->second.v)*0.02; //Calculates acceleration = difference in speed over a span of 20 mSeconds
              }
              Vehicle v = Vehicle(vlane, vs, vspeed, vaccel);
              road.update_traffic(vid, v);
            }
          }
          
          /////////////////////////////////////////////////
          // Behavior Planning
          /////////////////////////////////////////////////
          // from the road update() function we get back the next ideal state
          // Which we can then pull the target lane, velocity and acceleration from.
          Vehicle target = road.update();
          
          cout << ">>> Next State: " << target.state << " lane: " << target.lane << " speed: " << target.v << endl << endl;
          
          /////////////////////////////////////////////////
          // Trajectory Planning
          /////////////////////////////////////////////////
          
          int prev_size = previous_path_x.size();
          
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          // Increase/decrease velocity until we are in range of our target velocity
          ref_vel += target.a;
          
          // Make sure we never go faster than the speed limit
          if(ref_vel > 49.5) {
            ref_vel = 48;
          }
          
          // Check if any cars are next to us within a certain buffer before allowing a lane change.
          if(target.state.compare("LCL") == 0 || target.state.compare("LCR") == 0) {
            if(road.check_lane_clear(target.lane)) {
              lane = target.lane;
              road.update_ego_state(target.state);
            }
          } else {
            road.update_ego_state(target.state);
          }
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate them with a spilne and fill in the gaps
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Reference the x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous size is almost empty, use the car as starting point
          if(prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's endpoint as a starting reference
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Do a Shift and then a rotation to move to the origin (ego/car's reference point)
          // Shift the car reference angle to 0 degrees
          for(int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0-ref_yaw));
          }
          
          // create a spline
          tk::spline s;
          
          //set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          //define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //start with all of the previous path points from last time (this helps give a smooth transition)
          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //calculate how to break up spline points so that we travel at our desired reference  velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          // fill up the rest of our path planner after fillig it with previous points, here we will always output 50 points
          for(int i = 0; i <= 50 - previous_path_x.size(); i++) {
            double N = target_dist/(0.02*ref_vel/2.24); //Devide by 2.24 to get M/S, ref_vel was in MPH
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //rotate back to normal after rotating it earlier: Rotate and then shift from Car's reference to map reference.
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          json msgJson;

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
