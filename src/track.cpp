#include "track.h"

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

using namespace std;

const double Track::max_s = 6945.55405474;
const double Track::lane_width = 4.0;
const int Track::leftmost_lane = 0;
const int Track::rightmost_lane = 2;

constexpr double pi() { return M_PI; }

struct Track::splines {
    tk::spline x;
    tk::spline y;
    tk::spline dx;
    tk::spline dy;
};

Track::Track()
{
    string map_file_ = "../data/highway_map.csv";

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
        _map_waypoints_x.push_back(x);
        _map_waypoints_y.push_back(y);
        _map_waypoints_s.push_back(s);
        _map_waypoints_dx.push_back(d_x);
        _map_waypoints_dy.push_back(d_y);
    }

    _splines = new struct splines;

}

double Track::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/* Set local fine waypoints and fit splines to be used by more accurate co-ordinate transformation functions */
void Track::setLocality(int next_wp)
{
    enum {
        WAYPOINTS_SPAN_FORWARD = 5,
        WAYPOINTS_SPAN_BACKWARD = 2,
    };

    const int num_waypoints = _map_waypoints_s.size();
    vector<double> waypoints_x, waypoints_y, waypoints_s, waypoints_dx, waypoints_dy;

    _local_s_correction = 0.0;
    _local_s_transition = max_s;

    double _s = 0.0;

    /* Collect waypoints near the given way point */
    _first_local_wp = (next_wp + num_waypoints - WAYPOINTS_SPAN_BACKWARD) % num_waypoints;
    _last_local_wp = (next_wp + WAYPOINTS_SPAN_FORWARD) % num_waypoints;

    for (int i = -WAYPOINTS_SPAN_BACKWARD; i <= WAYPOINTS_SPAN_FORWARD; ++i) {
        int wp = i + next_wp + num_waypoints;

        while(wp >= num_waypoints)
            wp -= num_waypoints;

        double map_s = _map_waypoints_s[wp];

        /* correct s for wrap around the track */
        if (map_s < _s) {
            _local_s_correction = max_s;
            _local_s_transition = map_s;
        }

        _s = map_s;

        waypoints_s.push_back(map_s + _local_s_correction);
        waypoints_x.push_back(_map_waypoints_x[wp]);
        waypoints_y.push_back(_map_waypoints_y[wp]);
        waypoints_dx.push_back(_map_waypoints_dx[wp]);
        waypoints_dy.push_back(_map_waypoints_dy[wp]);
    }


    /* fit splines for x,y,dx,dy against s */
    _splines->x.set_points(waypoints_s, waypoints_x);
    _splines->y.set_points(waypoints_s, waypoints_y);
    _splines->dx.set_points(waypoints_s, waypoints_dx);
    _splines->dy.set_points(waypoints_s, waypoints_dy);


    /* generate finer waypoints in the visinity */
    const double s_delta = 0.5;

    _local_waypoints_s.clear();
    _local_waypoints_x.clear();
    _local_waypoints_y.clear();
    _local_waypoints_dx.clear();
    _local_waypoints_dy.clear();

    for (double s = waypoints_s[0]; s < waypoints_s[waypoints_s.size()-1] ; s += s_delta) {
        _local_waypoints_s.push_back(s);
        _local_waypoints_x.push_back(_splines->x(s));
        _local_waypoints_y.push_back(_splines->y(s));
        _local_waypoints_dx.push_back(_splines->dx(s));
        _local_waypoints_dy.push_back(_splines->dy(s));
    }
}


double Track::getD(int lane)
{
    return lane*lane_width + lane_width/2;
}

int Track::getLane(double d)
{
    double _d = d/lane_width;
    double lane;

    modf(_d, &lane);

    return static_cast<int>(lane);
}


void Track::setLocality(double x, double y, double theta)
{
    int next_wp = NextWaypoint(x,y, theta, _map_waypoints_x, _map_waypoints_y);
    setLocality(next_wp);
}

void Track::setLocality(double s)
{
    int next_wp = NextWaypoint(s, _map_waypoints_s);
    setLocality(next_wp);
}

/* More accurate transformation from XY to Frenet using interpolated local waypoints */
vector<double> Track::getFrenet(double x, double y, double theta, bool fine, bool localize)
{
    if (fine) {
        if (localize)
            setLocality(x, y, theta);

        vector<double> sd = getFrenet(x, y, theta, _local_waypoints_x, _local_waypoints_y);

        sd[0] += _map_waypoints_s[_first_local_wp];

        while (sd[0] > max_s)
            sd[0] -= max_s;

        return sd;
    } else {
        return getFrenet(x, y, theta, _map_waypoints_x, _map_waypoints_y);
    }
}

/* More accurate transformation from Frenet to XY using splines of interpolated local waypoints */
vector<double> Track::getXY(double s, double d, bool fine, bool localize)
{
    while (s >= max_s)
        s -= max_s;


    if (fine) {
        if (localize)
            setLocality(s);

        if (s >= _local_s_transition && s <= _map_waypoints_s[_last_local_wp])
            s += _local_s_correction;

        return {_splines->x(s) + d*_splines->dx(s), _splines->y(s) + d*_splines->dy(s)};
    } else {
        return getXY(s, d, _map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
    }
}

/* More accurate unit normal parallel to d vector using splines of interpolated local waypoints */
vector<double> Track::getDxDy(double s, bool fine, bool localize)
{
    while (s >= max_s)
        s -= max_s;

    if (fine) {
        if (localize)
            setLocality(s);

        vector<double> dxdy = {_splines->dx(s), _splines->dy(s)};
        double magnitude = sqrt(dxdy[0]*dxdy[0] + dxdy[1]*dxdy[1]);

        // normalization
        dxdy[0] /= magnitude;
        dxdy[1] /= magnitude;

        return dxdy;
    } else {
        int next_wp = NextWaypoint(s, _map_waypoints_s);
        return {_map_waypoints_dx[next_wp], _map_waypoints_dy[next_wp]};
    }
}

/* More accurate unit normal perpendicular to d vector using splines of interpolated local waypoints */
vector<double> Track::getSxSy(double s, bool fine, bool localize)
{
    vector<double> dxdy = getDxDy(s, fine, localize);
    return {-dxdy[1], dxdy[0]}; // rotate normal d vector by 90 degrees
}


/* Original co-ordinate transfer functions from starte code */

int Track::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int Track::NextWaypoint(double s, const vector<double> &maps_s)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  return (prev_wp+1)%maps_s.size();
}

int Track::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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



vector<double> Track::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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


vector<double> Track::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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