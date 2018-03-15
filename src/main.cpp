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

using namespace std;
// for convenience
using json = nlohmann::json;


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
  
  // Our Vehicle
  Vehicle car;
  // start in lane 1;
  int lane = 1;
  // Have a reference velocity to target
  double ref_vel = 0; //mpg
  
  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // j[1] is the data JSON object
          
//          // Main car's localization Data
//          double x = j[1]["x"];
//          double y = j[1]["y"];
//          double s = j[1]["s"];
//          double d = j[1]["d"];
//          double yaw = j[1]["yaw"];
//          double speed = j[1]["speed"];
//
//          // Previous path data given to the Planner
//          vector<double> previous_path_x = j[1]["previous_path_x"];
//          vector<double> previous_path_y = j[1]["previous_path_y"];
//          // Previous path's end s and d values
//          double end_path_s = j[1]["end_path_s"];
//          double end_path_d = j[1]["end_path_d"];
          
          // update car values;
//          car.update(x, y, s, d, yaw, speed, previous_path_x, previous_path_y, end_path_s, end_path_d);
          car.update(j);

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = car.previous_path_x.size();

          if(prev_size > 0) {
            car.s = car.end_path_s;
          }

          bool too_close = false;

          //find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++) {
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2 + 4 * lane +2) && d > (2 + 4*lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size*0.02*check_speed); // if using previous points can project s value out
              //check s values greater than mine and s gap
              if((check_car_s > car.s) && ( (check_car_s - car.s) < 30)) {
                // do some logic here, lower reference velocity so we don't crash into the car in front of us,
                // could also flag to try to change lanes.
//                ref_vel = check_speed;
                too_close = true;
                if(lane > 0) {
                  lane = 0;
                }

              }
            }
          }

          // This works if there are cars or no cars in front of it.  If you're starting from 0 Velocity, it will
          // increase slowly.
          if(too_close) {
            ref_vel -= 0.224;
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224;
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate them with a spilne and fill in the gaps
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference the x,y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car.x;
          double ref_y = car.y;
          double ref_yaw = deg2rad(car.yaw);

          // If previous size is almost empty, use the car as starting point
          if(prev_size < 2) {
            // use two points that make the path tangent to the car
            double prev_car_x = car.x - cos(car.yaw);
            double prev_car_y = car.y - sin(car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car.y);
          }
          // use the previous path's endpoint as a starting reference
          else {
            ref_x = car.previous_path_x[prev_size - 1];
            ref_y = car.previous_path_y[prev_size - 1];

            double ref_x_prev = car.previous_path_x[prev_size - 2];
            double ref_y_prev = car.previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car.s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car.s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car.s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
          tk::spline spl;

          //set (x,y) points to the spline
          spl.set_points(ptsx, ptsy);

          //define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //start with all of the previous path points from last time (this helps give a smooth transition)
          for(int i = 0; i < car.previous_path_x.size(); i++) {
            next_x_vals.push_back(car.previous_path_x[i]);
            next_y_vals.push_back(car.previous_path_y[i]);
          }

          //calculate how to break up spline points so that we travel at our desired reference  velocity
          double target_x = 30.0;
          double target_y = spl(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          // fill up the rest of our path planner after fillig it with previous points, here we will always output 50 points
          for(int i = 0; i <= 50 - car.previous_path_x.size(); i++) {
            double N = target_dist/(0.02*ref_vel/2.24); //Devide by 2.24 to get M/S, ref_vel was in MPH
            double x_point = x_add_on + (target_x)/N;
            double y_point = spl(x_point);

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

//          vector<double> next_x_vals;
//          vector<double> next_y_vals;
//
//          double dist_inc = 0.5;  //0.5m
//          for(int i = 0; i < 50; i++) {
//            double next_s = car.s + (i + 1)*dist_inc;
//            double next_d = 6;
//            vector<double> nextXY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            next_x_vals.push_back(nextXY[0]);
//            next_y_vals.push_back(nextXY[1]);
//          }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
