#ifndef _TRACK_
#define _TRACK_


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
    static const double lane_width;
  
    double distance(double x1, double y1, double x2, double y2);

    void setLocality(double x, double y, double theta);
    void setLocality(double s);
    void setLocality(int next_wp);

    vector<double> getFrenet(double x, double y, double theta, bool fine = false, bool localize = false);
    vector<double> getXY(double s, double d, bool fine = false, bool localize = false);

    vector<double> getDxDy(double s, bool fine = false, bool localize = false);
    vector<double> getSxSy(double s, bool fine = false, bool localize = false);

    double getD(int lane);
    int getLane(double d);


private:

    int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
    int NextWaypoint(double s, const vector<double> &maps_s);
    int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
    vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

    vector<double> _map_waypoints_x;
    vector<double> _map_waypoints_y;
    vector<double> _map_waypoints_s;
    vector<double> _map_waypoints_dx;
    vector<double> _map_waypoints_dy;

    vector<double> _local_waypoints_x;
    vector<double> _local_waypoints_y;
    vector<double> _local_waypoints_s;
    vector<double> _local_waypoints_dx;
    vector<double> _local_waypoints_dy;

    int _first_local_wp;
    int _last_local_wp;

    double _local_s_correction = 0.0;
    double _local_s_transition = max_s;

    struct splines;
    struct splines * _splines;
};

#endif