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

// Calculate Euclidian distance between two points.
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Identify closest waypoint to (x,y).
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

// Identify next waypoint ahead on the track.
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
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

// Check if specified lane is clear for lane change
bool lane_clear(int lane, vector<vector<double>> sensor_fusion, int previous_path_size, double car_s, double forward_buffer, double backward_buffer)
{
  bool clear_to_pass = true;
  for (int j=0; j < sensor_fusion.size(); j++) 
  {
    float d = sensor_fusion[j][6];
    if (d > 4*lane && d < 4*(lane+1))
    {
      double vx = sensor_fusion[j][3];
      double vy = sensor_fusion[j][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[j][5];
      check_car_s += ((double)previous_path_size * 0.02 * check_speed);
      if ( (check_car_s > car_s-backward_buffer) && (check_car_s < car_s+forward_buffer))
      {
        clear_to_pass = false;
      }
    }
  }
  return clear_to_pass;
}

// *** MAIN *** //

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

  // *** SET UP DRIVE VARIABLES *** //

  int lane_number = 1;
  double ref_velocity = 0; // mph
  vector<double> last_3_vels(3);
  last_3_vels[0] = 0.0;
  last_3_vels[1] = 0.0;
  last_3_vels[2] = 0.0;
  const double FALLBACK_DURATION_SEC = 5;
  vector<clock_t> last_3_fbs(3);
  clock_t start_time = clock();
  last_3_fbs[0] = start_time;
  last_3_fbs[1] = start_time;
  last_3_fbs[2] = start_time;
  bool slow_down_mode = false;
  clock_t slow_down_start = start_time;
  const double SLOW_DOWN_DURATION_SEC = 10;

  // *** DATA FROM SIMULATOR *** //

  h.onMessage([&lane_number, &ref_velocity, &last_3_vels, &FALLBACK_DURATION_SEC, &last_3_fbs, 
                &slow_down_mode, &SLOW_DOWN_DURATION_SEC, &slow_down_start,
                &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    const double COLLISION_BUFFER = 30;
    const double ACCEL_INCREMENT = 0.224; // 2.5 m/s^2
    const double TARGET_VELOCITY = 49.5; // mph
    const int CLOCKS_PER_SEC_CORRECTED = 100000; // built-in value appears to be wrong by 10x

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

          // ** PREDICTION ** //
          // Check if we are getting too close to a car ahead of us
          // If so check whether the adjacent lane(s) are open for passing

          int previous_path_size = previous_path_x.size();
          if (previous_path_size > 0) 
          {
            car_s = end_path_s;
          }
          bool too_close = false;
          bool lane_0_clear = false;
          bool lane_1_clear = false;
          bool lane_2_clear = false;

          // look through all other cars on the road
          for (int i=0; i < sensor_fusion.size(); i++) 
          {
            // test if car is in our lane
            if (!lane_clear(lane_number, sensor_fusion, previous_path_size, car_s, COLLISION_BUFFER, 0))
            {
              too_close = true;
              // Examine if adjacent lane(s) are clear
              // From lanes 0 or 2 can only change to lane 1
              if ((lane_number == 0) || (lane_number == 2)) 
              {
                lane_1_clear = lane_clear(1, sensor_fusion, previous_path_size, car_s, COLLISION_BUFFER, COLLISION_BUFFER);
              }
              else 
              {
                lane_0_clear = lane_clear(0, sensor_fusion, previous_path_size, car_s, COLLISION_BUFFER, COLLISION_BUFFER);
                lane_2_clear = lane_clear(2, sensor_fusion, previous_path_size, car_s, COLLISION_BUFFER, COLLISION_BUFFER);
              }
            } 
          }
                    
          // ** BEHAVIOR ** //
          // Adjust state if we're too close to a vehicle or below our target speed.
          // If too close, first decelerate and then consider changing lanes if target lane is clear.
          // Target lane for lanes 0 and 2 is lane 1.
          // Target lane for lane 1 is lane 0 (first choice) or lane 2 (second choice).
          // If below our target speed, accelerate.
          // If we aren't in lane 1 and it's clear, move into it (it's the optimal lane overall).
          // If we're in "slow down mode" adjust speed accordingly.

          if (too_close) 
          // Following a vehicle too closely - slow down and try to change lanes.
          {
            ref_velocity -= ACCEL_INCREMENT;
            if ((lane_number == 0) || (lane_number == 2))
            {
              if (lane_1_clear) 
              {
                lane_number = 1;
                slow_down_mode = false;
              }
            } 
            else 
            {
              if (lane_0_clear) 
              {
                lane_number = 0;
                slow_down_mode = false;
              }
              else if (lane_2_clear) 
              {
                lane_number = 2;
                slow_down_mode = false;
              }
            }
          }
          else if (slow_down_mode)
          // In "slow down mode" so adjust speed accordingly.
          {
            if (0.8*TARGET_VELOCITY-ref_velocity>ACCEL_INCREMENT+0.001) ref_velocity += ACCEL_INCREMENT;
            else if (ref_velocity-0.8*TARGET_VELOCITY>ACCEL_INCREMENT+0.001) ref_velocity -= ACCEL_INCREMENT;
          }
          else if (ref_velocity < TARGET_VELOCITY)
          {
            ref_velocity += ACCEL_INCREMENT;
          }
          else if ((lane_number != 1) && lane_clear(1, sensor_fusion, previous_path_size, car_s, COLLISION_BUFFER, COLLISION_BUFFER))
          {
            lane_number = 1;
          }

          // ** BEHAVIOR FOR OSCILLATION ** //
          // Ego vehicle can oscillate while following a slower-moving car if it can't pass.
          // (This results from acceleration/deceleration method - car "fallbacks" are periodic.)
          // Can manage oscillating fallback by first detecting then going into temporary slowdown mode.
          // After set duration, emerge from slowdown mode. 
          // Hopefully traffic has moved on and ego vehicle can pass.

          clock_t this_time = clock() / CLOCKS_PER_SEC_CORRECTED;  // Maybe use simulator time instead of system time?

          // Track last three velocity values
          last_3_vels[2] = last_3_vels[1];
          last_3_vels[1] = last_3_vels[0];
          last_3_vels[0] = ref_velocity;

          // Test for "fallback" condition
          double dv1 = last_3_vels[0] - last_3_vels[1];
          double dv2 = last_3_vels[2] - last_3_vels[1];

          if ((fabs(dv1-ACCEL_INCREMENT)<0.001) && (fabs(dv2-ACCEL_INCREMENT)<0.001))
          {
            // timestamp this fallback
            last_3_fbs[2] = last_3_fbs[1];
            last_3_fbs[1] = last_3_fbs[0];
            last_3_fbs[0] = this_time;
            // examine spacing between last three fallbacks
            double dfb1 = (last_3_fbs[0] - last_3_fbs[1]);
            double dfb2 = (last_3_fbs[1] - last_3_fbs[2]);

            cout << "FB1 dur: " << dfb1 << "; FB2 dur: " << dfb2 << endl;

            if ((!slow_down_mode) && (dfb1 < FALLBACK_DURATION_SEC) && (dfb2 < FALLBACK_DURATION_SEC))
            {
              // fallbacks are close and we are oscillating so go into slow down mode
              slow_down_mode = true;
              slow_down_start = this_time;
              cout << "Entering slow down mode" << endl;
            }
          }

          // Test for exiting fallback condition
          if (slow_down_mode)
          {
            double dsdm = (this_time - slow_down_start);
            if (dsdm > SLOW_DOWN_DURATION_SEC)
            {
              slow_down_mode = false;
              cout << "Exiting slow down mode" << endl;
            }
          }

          // ** TRAJECTORY ** //

          // prepare future path vector
        	vector<double> next_x_vals;
        	vector<double> next_y_vals;

          // set up reference values
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // build points for spline
          vector<double> ptsx;
          vector<double> ptsy;

          // first get last two points from previous path
          // if fewer than 2 points, use current position and 1 meter behind
          if (previous_path_size > 1) 
          {
            // we have at least 2 previous path points
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            double ref_x_prev = previous_path_x[previous_path_size-2];
            double ref_y_prev = previous_path_y[previous_path_size-2];
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
          } 
          else 
          {
            // not enough previous path points
            ptsx.push_back(car_x - 1 * cos(car_yaw));
            ptsx.push_back(car_x);
            ptsy.push_back(car_y - 1 * sin(car_yaw));
            ptsy.push_back(car_y);
          }

          // get three future points
          double spline_future_step = 30; // each point is 25 meters further ahead
          for (int i=0; i < 3; i++) {
            double next_s = car_s + (i+1)*spline_future_step;
            double next_d = 2 + 4*lane_number;
            vector<double> xy = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            ptsx.push_back(xy[0]);
            ptsy.push_back(xy[1]);
          }

          // transform to car coordinates
          for (int i=0; i < ptsx.size(); i++) {
            double x2 = ptsx[i] - ref_x;  // shift origin
            double y2 = ptsy[i] - ref_y;  // shift orgin
            ptsx[i] = x2 * cos(0-ref_yaw) - y2 * sin(0-ref_yaw); // rotate
            ptsy[i] = x2 * sin(0-ref_yaw) + y2 * cos(0-ref_yaw); // rotate
          }

          // calculate spline
          tk::spline lane_spline;
          lane_spline.set_points(ptsx,ptsy);

          // put previous path points into path
          for (int i=0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate spacing of future path points
          double look_ahead_x = 30; 
          double look_ahead_y = lane_spline(look_ahead_x);
          double look_ahead_dist = sqrt(look_ahead_x*look_ahead_x + look_ahead_y*look_ahead_y);
            
          double N_steps = look_ahead_dist / (0.02 * ref_velocity / 2.24);
          double x_step = look_ahead_x / N_steps;
          double x_offset = 0;

          // calculate sample points along spline
          for (int i=0; i < 50 - previous_path_size; i++) {

            double x = x_offset + x_step;
            double y = lane_spline(x);
            x_offset = x;
            double x_ref = x;
            double y_ref = y;
              
            // transform back to global coords
            x = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw) + ref_x;
            y = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw) + ref_y;

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

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
