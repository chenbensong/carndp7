#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  // Init lane and reference velocity.
  int lane = 1;
  double rv = 0;
  // Define what is "close" (within 25 meters)
  int CLOSE_RANGE = 25;
  
  h.onMessage([&lane, &rv,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // Find and change lanes as needed.
          
          // Nearby car detectors.
          bool front_car = false;
				  bool left_car = false;
				  bool right_car = false;

          // Check history.
          int prev_size = previous_path_x.size();
          if(prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Check sensor fusion data.
          for(int i = 0; i < sensor_fusion.size(); ++i) {
            float d = sensor_fusion[i][6];
			  		int curr_lane = -1;
						if ( d > 0 && d < 4 ) {
              curr_lane = 0;
						} else if ( d >= 4 && d < 8 ) {
							curr_lane = 1;
						} else if ( d >= 8 && d < 12 ) {
							curr_lane = 2;
						}
						if (curr_lane < 0) {
							continue;
						}

						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];
						double check_speed = sqrt(vx * vx + vy * vy);
						double check_car_s = sensor_fusion[i][5];

            // Project using previous points.
						check_car_s += ((double)prev_size * check_speed / 50.0);
						
            // Detect front cars in all lanes.
						if((check_car_s > car_s) && ((check_car_s - car_s) < CLOSE_RANGE)) {
							if (curr_lane == lane) {
								front_car = true;
							} else if (curr_lane < lane) {
								left_car = true;
							} else if (curr_lane > lane) {
								right_car = true;
							}
						}

            // Detect rear cars in neighboring lanes.
						if((curr_lane != lane) &&(check_car_s < car_s) && ((check_car_s - car_s) > -CLOSE_RANGE)) {
							if (curr_lane < lane) {
								left_car = true;
							} else if (curr_lane > lane) {
						  	right_car = true;
							}
						}
          }

          // Adjust speed and lane.
          if(front_car) {  // Slow down 
            ref_vel -= 0.2;  // Decrease 10 miles per hour.
			  		if ( !left_car && lane > 0 ) {
							--lane; // Switch left
						} else if ( !right_car && lane != 2 ) {
							++lane; // Switch right.
						}
					} else if(ref_vel < 49.8) {  // Upper bound is 50 mph - accelerate.
            rv += 0.2;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
