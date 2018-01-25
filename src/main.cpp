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


#define PREVIOUS_PATH_OVERLAP    (25)
#define TRACK_LENGTH             (6945.55405474)

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void print_vector(vector<double>& vec, const string& name, int n=0)
{
  int start = 0;
  int end = vec.size();

  std::cout << name << ' ';

  if (n > 0) {
    end = std::min(n,end);
  } else if (n < 0) {
    start = std::max(end+n, start);
  }

  for (int i=start; i < end; i++)
    std::cout << vec[i] << ' ';

  std::cout << std::endl;
}

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

int NextWaypoint(double s, const vector<double> &maps_s)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  return (prev_wp+1)%maps_s.size();
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> _getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y,
  const vector<double> &maps_dx, const vector<double> &maps_dy)
{
  enum {
    WAYPOINTS_SPAN = 4,
  };

  const int num_waypoints = maps_s.size();
  vector<double> waypoints_x, waypoints_y, waypoints_s, waypoints_dx, waypoints_dy;

  int next_wp = NextWaypoint(s, maps_s);

  double s_correction = 0.0;
  double s_transition = TRACK_LENGTH;

  double _s = 0.0;
  int wp = 0;

  for (int i = -WAYPOINTS_SPAN; i <= WAYPOINTS_SPAN; ++i) {
    wp = i + next_wp + num_waypoints;

    while(wp >= num_waypoints)
      wp -= num_waypoints;

    double map_s = maps_s[wp];

    if (map_s < _s) {
      s_correction = TRACK_LENGTH;
      s_transition = map_s;
    }

    _s = map_s;

    waypoints_s.push_back(map_s + s_correction);
    waypoints_x.push_back(maps_x[wp]);
    waypoints_y.push_back(maps_y[wp]);
    waypoints_dx.push_back(maps_dx[wp]);
    waypoints_dy.push_back(maps_dy[wp]);
  }

  tk::spline x_spline, y_spline, dx_spline, dy_spline;

  x_spline.set_points(waypoints_s, waypoints_x);
  y_spline.set_points(waypoints_s, waypoints_y);
  dx_spline.set_points(waypoints_s, waypoints_dx);
  dy_spline.set_points(waypoints_s, waypoints_dy);


  if (s >= s_transition && s <= maps_s[wp ? wp-1 : num_waypoints-1])
    s += s_correction;

  return {x_spline(s) + d*dx_spline(s), y_spline(s) + d*dy_spline(s)};
}




  int lane = 1;
  double ref_vel = 0; // mph

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

  double sd[][2]={{0, 0},{120.7,10},{6875,-10},{6920,0}};
  //double sd[][2]={{384, 0},{390.7,0},{745,0},{760,0}};

  for (int i=0; i < sizeof(sd)/sizeof(sd[0]); ++i) {
    vector<double> xy = getXY(sd[i][0], sd[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::cout << "s:" << sd[i][0] << " d:" << sd[i][1] <<" x:" << xy[0] << " y:" << xy[1] << std::endl;
  }

  std::cout << std::endl;

  for (int i=0; i < sizeof(sd)/sizeof(sd[0]); ++i) {
    vector<double> xy = _getXY(sd[i][0], sd[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
    std::cout << "s:" << sd[i][0] << " d:" << sd[i][1] <<" x:" << xy[0] << " y:" << xy[1] << std::endl;
  }

  //exit(0);


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();
            int path_overlap = std::min(PREVIOUS_PATH_OVERLAP, prev_size);

            std::cout << "v:" << car_speed << " x:" << car_x << " y:" << car_y << " yaw:" << car_yaw  << " s:" << car_s << " d:" << car_d << " prev size:" << prev_size << std::endl;


            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (path_overlap < 2) {
              double prev_car_x = car_x - cos(ref_yaw);
              double prev_car_y = car_y - sin(ref_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              ref_x = previous_path_x[path_overlap-1];
              ref_y = previous_path_y[path_overlap-1];

              double ref_x_prev = previous_path_x[path_overlap-2];
              double ref_y_prev = previous_path_y[path_overlap-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);              
            }

            double init_x = ptsx[1];
            double init_y = ptsy[1];

            vector<double> init_frenet = getFrenet(init_x, init_y, ref_yaw, map_waypoints_x, map_waypoints_y);

            double init_s = init_frenet[0];
            double init_d = init_frenet[1];

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            for(int i = 0; i < path_overlap; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double dist_inc = 0.5;
            for(int i = 0; i < 50-path_overlap; i++)
            {
              double next_s = init_s + (i+1)*dist_inc;
              double next_d = 6;

              vector<double> xy = _getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }

            print_vector(next_x_vals, "next_x_vals", -5);
            print_vector(next_y_vals, "next_y_vals", -5);

            std::cout << std::endl;

            json msgJson;


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
