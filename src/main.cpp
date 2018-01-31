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

using namespace std;

using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// for convenience
using json = nlohmann::json;


#define PREVIOUS_PATH_OVERLAP    (25)
#define TRACK_LENGTH             (6945.55405474)

#define DT_MOTION                (0.02)
#define N_POINTS_MOTION          (50)

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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> _getFrenet(double x, double y, double theta, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y,
  const vector<double> &maps_dx, const vector<double> &maps_dy)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  
  enum {
    WAYPOINTS_SPAN = 4,
  };

  const int num_waypoints = maps_s.size();
  vector<double> waypoints_x, waypoints_y, waypoints_s, waypoints_dx, waypoints_dy;

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

  const double s_delta = 0.5;

  vector<double> fine_maps_x, fine_maps_y;

  for (double s = waypoints_s[0]; s < waypoints_s[waypoints_s.size()-1] ; s += s_delta) {
    fine_maps_x.push_back(x_spline(s));
    fine_maps_y.push_back(y_spline(s));
  }

  vector<double> _sd = getFrenet(x, y, theta, fine_maps_x, fine_maps_y);

  _sd[0] += waypoints_s[0];
  if (_sd[0] > TRACK_LENGTH)
    _sd[0] -= TRACK_LENGTH;

  return _sd;
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
  int next_wp = NextWaypoint(s, maps_s);

  enum {
    WAYPOINTS_SPAN = 4,
  };

  const int num_waypoints = maps_s.size();
  vector<double> waypoints_x, waypoints_y, waypoints_s, waypoints_dx, waypoints_dy;

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
  Map * track;
  Trajectory * trajectory;
  Motion * motion;

  int count_i = 0;


            vector<double> next_s_vals;
            vector<double> next_d_vals;

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

            int prev_size = previous_path_x.size();
            int path_overlap = std::min(PREVIOUS_PATH_OVERLAP, prev_size); 

            std::cout << "v:" << car_speed << " x:" << car_x << " y:" << car_y << " yaw:" << car_yaw  << " s:" << car_s << " d:" << car_d << " prev size:" << prev_size << std::endl;

            vector<double> previous_path_s;
            vector<double> previous_path_d;

            for (int i = N_POINTS_MOTION - previous_path_x.size(); i < N_POINTS_MOTION; ++i) {
              previous_path_s.push_back(next_s_vals[i]);
              previous_path_d.push_back(next_d_vals[i]);
            }

            motion->telemetry(car_x, car_y, car_s, car_d, car_yaw, car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d);




            print_vector(previous_path_s, "previous_path_s-", 5);
            //print_vector(previous_path_x, "previous_path_x-", 5);
            //print_vector(previous_path_y, "previous_path_y-", 5);

            print_vector(previous_path_s, "-previous_path_s", -5);
            //print_vector(previous_path_x, "-previous_path_x", -5);
            //print_vector(previous_path_y, "-previous_path_y", -5);

            print_vector(previous_path_s, "-previous_path_s", path_overlap-4, path_overlap+4);

           
            if (prev_size == 0) {
              end_path_s = car_s;
            }

            bool too_close = false;

            for (int i=0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];

              if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += (double)prev_size*0.02*check_speed;

                if ((check_car_s > end_path_s) && ((check_car_s-end_path_s)<30)) {
                  too_close = true;
                }
              }
            }

            if (too_close) {
              ref_vel -= 0.1;
            } else if (ref_vel < 22) {
              ref_vel += 0.1;
            }



            vector<double> s_i(3), d_i(3);
            vector<double> s_i_(3), d_i_(3);

            vector<double> s_f(3), d_f(3);

            vector<double> x_i(3), y_i(3);
            vector<double> x_i_(3), y_i_(3);

            double yaw_i;

            if (path_overlap < 3) {
              yaw_i = deg2rad(car_yaw);

              x_i[0] = car_x;
              x_i[1] = car_speed * cos(yaw_i);
              x_i[2] = 0;

              y_i[0] = car_y;
              y_i[1] = car_speed * sin(yaw_i);
              y_i[2] = 0;

              x_i_[0] = car_x - x_i[1]*DT_MOTION;
              x_i_[0] = car_x - cos(yaw_i);
              x_i_[1] = x_i[1];
              x_i_[2] = 0;

              y_i_[0] = car_y - y_i[1]*DT_MOTION;
              y_i_[0] = car_y - sin(yaw_i);
              y_i_[1] = y_i[1];
              y_i_[2] = 0;

              s_i[0] = car_s;
              s_i[1] = car_speed;
              s_i[2] = 0;

              d_i[0] = car_d;
              d_i[1] = 0;
              d_i[2] = 0;

              s_i_[0] = car_s - 1;
              d_i_[0] = car_d;

              track->setLocality(x_i[0], y_i[0], yaw_i);
            } else {
              vector<double> x_i__(3), y_i__(3);
              vector<double> s_i__(3), d_i__(3);

              x_i[0] = previous_path_x[path_overlap-1];
              y_i[0] = previous_path_y[path_overlap-1];

              x_i_[0] = previous_path_x[path_overlap-2];
              y_i_[0] = previous_path_y[path_overlap-2];

              x_i__[0] = previous_path_x[path_overlap-3];
              y_i__[0] = previous_path_y[path_overlap-3];


              x_i[1] = (x_i[0] - x_i_[0]) / DT_MOTION;
              y_i[1] = (y_i[0] - y_i_[0]) / DT_MOTION;

              x_i_[1] = (x_i_[0] - x_i__[0]) / DT_MOTION;
              y_i_[1] = (y_i_[0] - y_i__[0]) / DT_MOTION;

              x_i[2] = (x_i[1] - x_i_[1]) / DT_MOTION;
              y_i[2] = (y_i[1] - y_i_[1]) / DT_MOTION;

              yaw_i = atan2(y_i[0] - y_i_[0], x_i[0] - x_i_[0]);  

              track->setLocality(x_i[0], y_i[0], yaw_i);

              s_i[0] = previous_path_s[path_overlap-1];
              d_i[0] = previous_path_d[path_overlap-1]; 

              s_i_[0] = previous_path_s[path_overlap-2];
              d_i_[0] = previous_path_d[path_overlap-2]; 

              s_i__[0] = previous_path_s[path_overlap-3];
              d_i__[0] = previous_path_d[path_overlap-3]; 


              s_i[1] = (s_i[0] - s_i_[0]) / DT_MOTION;
              d_i[1] = (d_i[0] - d_i_[0]) / DT_MOTION;

              s_i_[1] = (s_i_[0] - s_i__[0]) / DT_MOTION;
              d_i_[1] = (d_i_[0] - d_i__[0]) / DT_MOTION;

              s_i[2] = (s_i[1] - s_i_[1]) / DT_MOTION;
              d_i[2] = (d_i[1] - d_i_[1]) / DT_MOTION;

            }

            //print_vector(s_i, "s_i");
            //print_vector(s_i_, "s_i_");

            print_vector(s_i, "s_i");
            print_vector(d_i, "d_i");

            print_vector(x_i, "x_i");
            print_vector(y_i, "y_i");

            vector<double> tx_i = motion->getInitX();
            vector<double> ty_i = motion->getInitY();
            vector<double> ts_i = motion->getInitS();
            vector<double> td_i = motion->getInitD();

            print_vector(ts_i, "M s_i");
            print_vector(td_i, "M d_i");

            print_vector(tx_i, "M x_i");
            print_vector(ty_i, "M y_i");


            s_f[0] = s_i[0] + ref_vel * TRAJECTORY_HORIZON;
            d_f[0] = 2 + 4*lane;

            trajectory->generateCVTrajectory(s_i, d_i, s_f, d_f, TRAJECTORY_HORIZON);
#if 1
            motion->generateMotion(trajectory, track);
#else
            motion->generateMotion(trajectory, track, lane, ref_vel);
#endif
            vector<double> ptsx;
            vector<double> ptsy;
            vector<double> ptss;
            vector<double> ptsd;
        
            ptss.push_back(s_i_[0]);
            ptss.push_back(s_i[0]);

            ptsd.push_back(d_i_[0]);
            ptsd.push_back(d_i[0]);

            ptsx.push_back(x_i_[0]);
            ptsx.push_back(x_i[0]);

            ptsy.push_back(y_i_[0]);
            ptsy.push_back(y_i[0]);  


            for (int i = 30; i <= 90; i += 30) {
              //vector<double> next_wp = getXY(car_s+i, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
#if 1
              double t = trajectory->timeToDestination(s_i[0]+i);

              double next_wp_s = trajectory->s(t);
              double next_wp_d = trajectory->d(t);
#else
              double next_wp_s = s_i[0]+i;
              double next_wp_d = 2+4*lane;
#endif
              vector<double> next_wp = track->getXY(next_wp_s, next_wp_d);

              ptss.push_back(next_wp_s);
              ptsd.push_back(next_wp_d);

              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
            }

            cout <<endl;

            print_vector(ptss, "ptss");
            print_vector(ptsd, "ptsd");
            print_vector(ptsx, "ptsx");
            print_vector(ptsy, "ptsy");

#if 1
            tk::spline splinex;
            splinex.set_points(ptss, ptsx);

            tk::spline spliney;
            spliney.set_points(ptss, ptsy);
#else
            double ref_s = s_i[0];
            double ref_x = x_i[0];
            double ref_y = y_i[0];
            double ref_yaw = yaw_i;

            for (int i=0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptss[i] = ptss[i] - ref_s;
              ptsx[i] = shift_x*cos(ref_yaw) + shift_y*sin(ref_yaw);
              ptsy[i] = -shift_x*sin(ref_yaw) + shift_y*cos(ref_yaw);
            }

            print_vector(ptss, "car ptss");
            print_vector(ptsx, "car ptsx");
            print_vector(ptsy, "car ptsy");


            tk::spline s;
            s.set_points(ptsx, ptsy);
#endif
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            next_s_vals.clear();
            next_d_vals.clear();


            for(int i = 0; i < path_overlap; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
              next_s_vals.push_back(previous_path_s[i]);
              next_d_vals.push_back(previous_path_d[i]);
            }

#if 1
            for(int i = 0; i < N_POINTS_MOTION-path_overlap; i++) {
              double t = (i+1) * DT_MOTION;

              double s_point = trajectory->s(t);// - ref_s;//t * ref_vel;
              double d_point = trajectory->d(t);
              double x_point = splinex(s_point);
              double y_point = spliney(s_point);

              next_s_vals.push_back(s_point);
              next_d_vals.push_back(d_point);
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
#else
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;
            double N = (target_dist/(DT_MOTION*ref_vel));


            for(int i = 0; i < N_POINTS_MOTION - path_overlap; i++) {
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            for(int i = path_overlap; i < N_POINTS_MOTION; i++) {

              double x_point_map = ref_x + next_x_vals[i]*cos(ref_yaw) - next_y_vals[i]*sin(ref_yaw);
              double y_point_map = ref_y + next_x_vals[i]*sin(ref_yaw) + next_y_vals[i]*cos(ref_yaw);

              double theta = (i > 0) ? atan2(y_point_map - next_y_vals[i-1], x_point_map - next_x_vals[i-1]) : yaw_i; 
              vector<double> sd = track->getFrenet(x_point_map, y_point_map, theta);
              next_s_vals.push_back(sd[0]);
              next_d_vals.push_back(sd[1]);

              next_x_vals[i] = x_point_map;
              next_y_vals[i] = y_point_map;
            }
#endif

            vector<double> _next_x_vals;
            vector<double> _next_y_vals;

            motion->getMotion(_next_x_vals, _next_y_vals);

            print_vector(next_s_vals, "next_s_vals-", 10);
            print_vector(next_x_vals, "next_x_vals-", 10);
            print_vector(next_y_vals, "next_y_vals-", 10);

            print_vector(_next_x_vals, "M next_x_vals-", 10);
            print_vector(_next_y_vals, "M next_y_vals-", 10);

            print_vector(next_s_vals, "-next_s_vals", -10);
            print_vector(next_x_vals, "-next_x_vals", -10);
            print_vector(next_y_vals, "-next_y_vals", -10);

            print_vector(_next_x_vals, "M next_x_vals", -10);
            print_vector(_next_y_vals, "M next_y_vals", -10);


            count_i++;

            //if (count_i >= 2)
            //exit(0);

            std::cout << std::endl;

            json msgJson;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = _next_x_vals;
          	msgJson["next_y"] = _next_y_vals;

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
