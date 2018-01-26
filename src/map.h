
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
//#include "spline.h"

using namespace std;

class Map
{
public:
	Map();

    static const double max_s;
  
    double distance(double x1, double y1, double x2, double y2);

    void setLocality(double x, double y, double theta);
    void setLocality(double s);
    void setLocality(int next_wp);

    vector<double> getFrenet(double x, double y, double theta);
    vector<double> getXY(double s, double d);


private:

    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
    int NextWaypoint(double s, const vector<double> &maps_s);
    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


    vector<double> local_waypoints_x;
    vector<double> local_waypoints_y;
    vector<double> local_waypoints_s;
    vector<double> local_waypoints_dx;
    vector<double> local_waypoints_dy;

    int first_local_wp;
    int last_local_wp;

    double local_s_correction = 0.0;
    double local_s_transition = max_s;

    struct splines;
    struct splines * _splines;
};