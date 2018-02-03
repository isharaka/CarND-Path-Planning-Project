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

#include "map.h"
#include "trajectory.h"
#include "motion.h"
#include "prediction.h"
#include "car.h"
#include "behavior.h"

using namespace std;

using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// for convenience
using json = nlohmann::json;


#define DT_TRAJECTORY            (0.2)
#define N_POINTS_TRAJECTORY      (25)
#define TRAJECTORY_HORIZON       (DT_TRAJECTORY * N_POINTS_TRAJECTORY)


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void print_vector(vector<double>& vec, const string& name, int n=0)
{
  int start = 0;
  int end = vec.size();


  if (n > 0) {
    end = std::min(n,end);
  } else if (n < 0) {
    start = std::max(end+n, start);
  }
  std::cout << name << " (" << vec.size() <<' '<< start << ' ' << end << ") " ;

  for (int i=start; i < end; i++)
    std::cout << vec[i] << ' ';

  std::cout << std::endl;
}

void print_vector(vector<double>& vec, const string& name, int b, int e)
{
  int start = std::max(std::min(b, (int)vec.size()),0);
  int end = std::max(std::min(std::max(b,e), (int)vec.size()),0);

  std::cout << name << " (" << vec.size() <<' '<< start << ' ' << end << ") " ;

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




  Map * track;
  Trajectory * trajectory;
  Motion * motion;
  Prediction * prediction;
  Behavior * behavior;

  int count_i = 0;


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

  track = new Map();
  trajectory = new Trajectory();
  motion = new Motion();
  prediction = new Prediction();
  behavior = new Behavior();

  double sd[][2]={{0, 0},{120.7,10},{6875,-10},{6920,0}};
  //double sd[][2]={{384, 0},{390.7,0},{745,0},{760,0}};

  for (int i=0; i < sizeof(sd)/sizeof(sd[0]); ++i) {
    vector<double> xy = getXY(sd[i][0], sd[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::cout << "s:" << sd[i][0] << " d:" << sd[i][1] <<" x:" << xy[0] << " y:" << xy[1] << std::endl;
  }

  std::cout << std::endl;

  for (int i=0; i < sizeof(sd)/sizeof(sd[0]); ++i) {
    //vector<double> xy = _getXY(sd[i][0], sd[i][1], map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
    //track->setLocality(sd[i][0]);
    vector<double> xy = track->getXY(sd[i][0], sd[i][1]);
    //vector<double> dxdy = track->getDxDy(sd[i][0]);
    std::cout << "s:" << sd[i][0] << " d:" << sd[i][1] <<" x:" << xy[0] << " y:" << xy[1] <<
      /*" dx:" << dxdy[0] << " dy:" << dxdy[1] << " n:" << sqrt(dxdy[0]*dxdy[0] + dxdy[1]*dxdy[1]) <<*/ std::endl;
  }

  std::cout << std::endl;

  double xy[][2]={{1025,1157},{1370,1185},{1025 + 5*0.4,1157-5*0.9},{1370+4*0.1,1185+4*1}};
  double a[4];

  a[0] = atan2(1157.81 - 1145.318, 1025.028 - 995.2703);
  a[1] = atan2(1185.671 - 1188.307, 1369.225 - 1340.477);
  a[2] = atan2(1157.81 - 1145.318, 1025.028 - 995.2703);
  a[3] = atan2(1185.671 - 1188.307, 1369.225 - 1340.477);

  for (int i=0; i < sizeof(xy)/sizeof(xy[0]); ++i) {
    vector<double> sd = getFrenet(xy[i][0], xy[i][1], a[i], map_waypoints_x, map_waypoints_y);
    std::cout << "x:" << xy[i][0] << " y:" << xy[i][1] <<" s:" << sd[0] << " d:" << sd[1] << std::endl;
  }

  std::cout << std::endl;

  for (int i=0; i < sizeof(xy)/sizeof(xy[0]); ++i) {
    //vector<double> sd = _getFrenet(xy[i][0], xy[i][1], a[i], map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
    //track->setLocality(xy[i][0], xy[i][1], a[i]);
    vector<double> sd = track->getFrenet(xy[i][0], xy[i][1], a[i]);
    //vector<double> dxdy = track->getDxDy(sd[0]);
    std::cout << "x:" << xy[i][0] << " y:" << xy[i][1] <<" s:" << sd[0] << " d:" << sd[1] <<
      /*" dx:" << dxdy[0] << " dy:" << dxdy[1] << " n:" << sqrt(dxdy[0]*dxdy[0] + dxdy[1]*dxdy[1]) <<*/ std::endl;
  }

  vector<double> aa;
  aa.push_back(0);
  aa.push_back(1);

  cout << "aa[0] " << aa[0] << " aa[1] " << aa[1] << endl;

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
            car_speed = car_speed * 0.44704;

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

            std::cout << "v:" << car_speed << " x:" << car_x << " y:" << car_y << " yaw:" << car_yaw  << " s:" << car_s << " d:" << car_d << " prev size:" << previous_path_x.size() << std::endl << std::endl;

            // MOTION & TELEMETRY
            motion->telemetry(track, car_x, car_y, car_s, car_d, deg2rad(car_yaw), car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d);

            vector<double> x_i = motion->getInitX();
            vector<double> y_i = motion->getInitY();
            vector<double> s_i = motion->getInitS();
            vector<double> d_i = motion->getInitD();


            cout << "travel t:" << motion->getPreviousPathTravelTime() << " overlap:" << motion->getPreviousPathOverlapTime() <<
              " pre init s:" << motion->getPreviousInitS()[0] << " pre init d:" << motion->getPreviousInitD()[0] << 
              " pre s(0):" << trajectory->s(0)[0] << " pre s dot(0):" << trajectory->s(0)[1]  << endl;


            // PREDICTION
            double prediction_horizon_ego = motion->getPreviousPathTravelTime() + trajectory->time_horizon;
            Car ego = prediction->predict(prediction_horizon_ego, motion->getS(), motion->getD(), trajectory);

            double prediction_horizon_env = motion->getPreviousPathOverlapTime() + trajectory->time_horizon;
            map<int, Car> cars = prediction->predict(prediction_horizon_env, sensor_fusion); 
           

            // BEHAVIOR PLANNINGS
            struct Behavior::target target_behavior = behavior->generateBehavior(ego, cars, track);


            // TRAJECTORY GENERATION

#if 0
            vector<double> s_f(3), d_f(3);
            s_f[0] = s_i[0] + target_behavior.speed * trajectory->time_horizon;
            d_f[0] = 2 + 4*target_behavior.lane;

            trajectory->generateCVTrajectory(s_i, d_i, s_f, d_f, trajectory->time_horizon);
#else
            vector<double> s_f = { s_i[0] + (s_i[1] + target_behavior.speed) * trajectory->time_horizon / 2, target_behavior.speed, 0};
            vector<double> d_f = { track->getD(target_behavior.lane), 0, 0};

            print_vector(s_i, "s_i");
            print_vector(s_f, "s_f");
            print_vector(d_i, "d_i");
            print_vector(d_f, "d_f");

            print_vector(x_i, "x_i");
            print_vector(y_i, "y_i");

            trajectory->generateJMTrajectory(s_i, d_i, s_f, d_f, trajectory->time_horizon);

#endif


            // MOTION GENERATION
#if 1
            motion->generateMotion(trajectory, track);
#else
            motion->generateMotion(trajectory, track, lane, target_behavior.speed);
#endif

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            motion->getMotion(next_x_vals, next_y_vals);

            //print_vector(next_x_vals, "next_x_vals-", 10);
            //print_vector(next_y_vals, "next_y_vals-", 10);

            //print_vector(next_x_vals, "next_x_vals", -10);
            //print_vector(next_y_vals, "next_y_vals", -10);

            count_i++;

            //if (count_i >= 2)
            //exit(0);

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
