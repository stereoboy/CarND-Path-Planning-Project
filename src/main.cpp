#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<string> successor_states(vector<double> vehicle, string state, int lane, int lanes_available) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    double current_s = vehicle[0];
    double current_d = vehicle[1];

    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

float lane_speed(vector<double> vehicle, vector<vector<double>> sensor_fusion, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */

    double current_s = vehicle[0];
    double current_d = vehicle[1];

    double min_vel = 1000000;
    bool found = false;
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
      auto elem = sensor_fusion[i];
      int id = elem[0];
      double x = elem[1];
      double y = elem[2];
      double vx = elem[3];
      double vy = elem[4];
      double s  = elem[5];
      double d  = elem[6];
      double speed = sqrt(vx*vx + vy*vy);
      if ((d < 2+4*lane + 2) && (d > 2+4*lane - 2))
      {
        if ((current_s - 50 < s) && ( s < current_s + 300 ))
        {
          if (speed < min_vel)
          {
            found = true;
            min_vel = speed;
          }
        }
      }
    }

    //Found no vehicle in the lane
    if (found)
      return min_vel;
    return -1.0;
}

float inefficiency_cost(vector<double> vehicle, double ref_vel, string state, int current_lane, vector<vector<double>> sensor_fusion) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have slower traffic. 
    */
    int intended_lane;
    float final_lane = current_lane;

    if (state.compare("PLCL") == 0) {
        intended_lane = current_lane - 1;
    } else if (state.compare("PLCR") == 0) {
        intended_lane = current_lane + 1;
    } else {
        intended_lane = current_lane;
    }

    float proposed_speed_intended = lane_speed(vehicle, sensor_fusion, intended_lane);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = ref_vel;
    }

    float proposed_speed_final = lane_speed(vehicle, sensor_fusion, final_lane);
    if (proposed_speed_final < 0) {
        proposed_speed_final = ref_vel;
    }
    
    float cost = (2.0*ref_vel - proposed_speed_intended - proposed_speed_final)/ref_vel;

    return cost;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  int lane = 1;
  double ref_vel = 0.0; // 0 MPH - 0 m/s
#define SAFE_MARGIN_FRONT 30.0
#define MAX_REF_VEL 49.5 // 50 MPH - 22.352 m/s
#define SAFE_VEL_DIFF 0.224 // 0.224 MPH 0.1 m/2
  string state = "KL";

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

  h.onMessage([&lane, &ref_vel, &state, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            int prev_size = previous_path_x.size();
            double max_jerk = 10.0;
            double max_accT = 10.0;
            double max_accN = 10.0;
            bool too_close = false;

            for(int i = 0; i < sensor_fusion.size(); i++)
            {
              auto elem = sensor_fusion[i];
              int id = elem[0];
              double x = elem[1];
              double y = elem[2];
              double vx = elem[3];
              double vy = elem[4];
              double s  = elem[5];
              double d  = elem[6];
              double speed = sqrt(vx*vx + vy*vy);

              if ((d < 2+4*lane + 2) && (d > 2+4*lane - 2)) {
                if ((s > car_s) && (s - car_s) < SAFE_MARGIN_FRONT) {
                  too_close = true;
                }
              }
            }
#if 1
            if (too_close) {
              ref_vel -= SAFE_VEL_DIFF;
            } else if (ref_vel < MAX_REF_VEL) {
              ref_vel += SAFE_VEL_DIFF;
            }
#endif
            fprintf(stderr, "ref_vel: %f\n", ref_vel);
#if 0 
            vector<string> states = successor_states({car_s, car_d}, state, lane, 3);

            double min_cost = 100000000;

            for (int i = 0; i < states.size(); i++)
            {
              double cost = inefficiency_cost({car_s, car_d}, ref_vel, states[i], lane, sensor_fusion);
              if (cost < min_cost)
              {
                min_cost = cost;
                state = states[i];
              }
            }

            if (state.compare("LCL") == 0) {
              lane = lane - 1;
            } else if (state.compare("RCL") == 0) {
              lane = lane + 1;
            }
#endif

            // points for path-smoothing using spline
            vector<double> ptsx;
            vector<double> ptsy;

            // target terminal point to append in path array
            double ref_x;
            double ref_y;
            double prev_ref_x;
            double prev_ref_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_size < 2) { // first two state
              ref_x = car_x;
              ref_y = car_y;
              prev_ref_x = car_x - cos(car_yaw);
              prev_ref_y = car_y - sin(car_yaw);
            } else { // after
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              prev_ref_x = previous_path_x[prev_size - 2];
              prev_ref_y = previous_path_y[prev_size - 2];
            }

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
            auto next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            auto next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // rotate along ref_yaw for path-smoothing
            for (int i = 0; i < ptsx.size(); i++)
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
              ptsy[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

            for (int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
            for (int i = 0; i < 50 - prev_size; i++)
            {
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = (i + 1)*target_x/N;
              fprintf(stderr, "N:%f\n", N);
              fprintf(stderr, "x_points:%f\n", x_point);
              double y_point = s(x_point);

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
#if 0
            fprintf(stderr, "angle: %f\n", car_yaw);
//            int next_w_i = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
//            auto target = getXY(map_waypoints_s[next_w_i], car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//            fprintf(stderr, "previous_path_size: %ld\n", previous_path_x.size());
            double dist_inc = 0.2;
            for(int i = 0; i < 50; i++)
            {
              auto next = getXY(car_s + (dist_inc*(i+1)), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(next[0]);
              next_y_vals.push_back(next[1]);
            }
#endif
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
