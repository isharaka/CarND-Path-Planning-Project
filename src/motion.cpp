#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "motion.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);



#define N_POINTS_MOTION         50
#define PREVIOUS_PATH_OVERLAP   25
#define DT_MOTION               0.02

#define N_TRAJECTORY_WAYPOINTS  3

Motion::Motion():
    s_i(3), d_i(3), x_i(3), y_i(3), s_i_(3), d_i_(3), x_i_(3), y_i_(3), _previous_s_i(3), _previous_d_i(3)
{

}

void Motion::telemetry(
    Track * track,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    double car_speed,
    vector<double> previous_path_x,
    vector<double> previous_path_y,
    double end_path_s,
    double end_path_d
    )
{
    _car_x = car_x;
    _car_y = car_y;
    _car_s = car_s;
    _car_d = car_d;

    _car_yaw = car_yaw;
    _car_speed = car_speed;

    _previous_path_x = previous_path_x;
    _previous_path_y = previous_path_y;

    _end_path_s = end_path_s;
    _end_path_d = end_path_d;

    _previous_path_s.clear();
    _previous_path_d.clear();

    for (int i = N_POINTS_MOTION - _previous_path_x.size(); i < N_POINTS_MOTION; ++i) {
      _previous_path_s.push_back(_next_s_vals[i]);
      _previous_path_d.push_back(_next_d_vals[i]);
    }

    int prev_size = previous_path_x.size();

    if (prev_size == 0) {
        _end_path_s = car_s;
        _end_path_d = car_d;
    }

    _previous_s_i = s_i;
    _previous_d_i = d_i;

    _path_overlap = std::min((int)PREVIOUS_PATH_OVERLAP, prev_size);

    //print_vector(_previous_path_s, "_previous_path_s", PREVIOUS_PATH_OVERLAP-5, PREVIOUS_PATH_OVERLAP+5);

    calculateDerivatives(track);

}

void Motion::generateMotion(Trajectory * trajectory, Track * track)
{
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

    double wp_t = trajectory->time_horizon / N_TRAJECTORY_WAYPOINTS;

    for (int i = 1; i <= N_TRAJECTORY_WAYPOINTS; ++i) {
        double t = i * wp_t;

        double next_wp_s = trajectory->s(t)[0];
        double next_wp_d = trajectory->d(t)[0];

        vector<double> next_wp = track->getXY(next_wp_s, next_wp_d);
        //cout <<"t:" << t << " ";

        ptss.push_back(next_wp_s);
        ptsd.push_back(next_wp_d);

        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    //print_vector(ptss, "ptss");
    //print_vector(ptsd, "ptsd");
    //print_vector(ptsx, "ptsx");
    //print_vector(ptsy, "ptsy");


    tk::spline splinex;
    splinex.set_points(ptss, ptsx);

    tk::spline spliney;
    spliney.set_points(ptss, ptsy);

    _next_s_vals.clear();
    _next_d_vals.clear();
    _next_x_vals.clear();
    _next_y_vals.clear();


    for(int i = 0; i < _path_overlap; i++)
    {
        _next_x_vals.push_back(_previous_path_x[i]);
        _next_y_vals.push_back(_previous_path_y[i]);
        _next_s_vals.push_back(_previous_path_s[i]);
        _next_d_vals.push_back(_previous_path_d[i]);
    }

    for(int i = 0; i < N_POINTS_MOTION-_path_overlap; i++) {
        double t = (i+1) * DT_MOTION;

        double s_point = trajectory->s(t)[0];// - ref_s;//t * ref_vel;
        double d_point = trajectory->d(t)[0];
        double x_point = splinex(s_point);
        double y_point = spliney(s_point);

        _next_s_vals.push_back(s_point);
        _next_d_vals.push_back(d_point);
        _next_x_vals.push_back(x_point);
        _next_y_vals.push_back(y_point);
    }

    //print_vector(_next_s_vals, " _next_s_vals-", 10);
    //print_vector(_next_s_vals, "-_next_s_vals-", _path_overlap-5, _path_overlap+5);
    //print_vector(_next_s_vals, "-_next_s_vals ", -10);


}

void Motion::getMotion(vector<double>& next_x_vals, vector<double>& next_y_vals)
{
    next_x_vals = _next_x_vals;
    next_y_vals = _next_y_vals;
}


double Motion::calculateDerivatives(Track * track)
{
    if (_path_overlap < 3) {
        yaw_i = _car_yaw;

        x_i[0] = _car_x;
        x_i[1] = _car_speed * cos(yaw_i);
        x_i[2] = 0;

        y_i[0] = _car_y;
        y_i[1] = _car_speed * sin(yaw_i);
        y_i[2] = 0;

        x_i_[0] = _car_x - x_i[1]*DT_MOTION;
        x_i_[0] = _car_x - cos(yaw_i);
        x_i_[1] = x_i[1];
        x_i_[2] = 0;

        y_i_[0] = _car_y - y_i[1]*DT_MOTION;
        y_i_[0] = _car_y - sin(yaw_i);
        y_i_[1] = y_i[1];
        y_i_[2] = 0;

        s_i[0] = _car_s;
        s_i[1] = _car_speed;
        s_i[2] = 0;

        d_i[0] = _car_d;
        d_i[1] = 0;
        d_i[2] = 0;

        s_i_[0] = _car_s - 1;
        d_i_[0] = _car_d;

    } else {
        vector<double> x_i__(3), y_i__(3);
        vector<double> s_i__(3), d_i__(3);

        x_i[0] = _previous_path_x[_path_overlap-1];
        y_i[0] = _previous_path_y[_path_overlap-1];

        x_i_[0] = _previous_path_x[_path_overlap-2];
        y_i_[0] = _previous_path_y[_path_overlap-2];

        x_i__[0] = _previous_path_x[_path_overlap-3];
        y_i__[0] = _previous_path_y[_path_overlap-3];


        x_i[1] = (x_i[0] - x_i_[0]) / DT_MOTION;
        y_i[1] = (y_i[0] - y_i_[0]) / DT_MOTION;

        x_i_[1] = (x_i_[0] - x_i__[0]) / DT_MOTION;
        y_i_[1] = (y_i_[0] - y_i__[0]) / DT_MOTION;

        x_i[2] = (x_i[1] - x_i_[1]) / DT_MOTION;
        y_i[2] = (y_i[1] - y_i_[1]) / DT_MOTION;

        yaw_i = atan2(y_i[0] - y_i_[0], x_i[0] - x_i_[0]);  


        s_i[0] = _previous_path_s[_path_overlap-1];
        d_i[0] = _previous_path_d[_path_overlap-1]; 

        s_i_[0] = _previous_path_s[_path_overlap-2];
        d_i_[0] = _previous_path_d[_path_overlap-2]; 

        s_i__[0] = _previous_path_s[_path_overlap-3];
        d_i__[0] = _previous_path_d[_path_overlap-3]; 

        double delta_s = s_i[0] - s_i_[0] + track->max_s;

        while(delta_s > track->max_s)
            delta_s -= track->max_s;

        s_i[1] = (s_i[0] - s_i_[0]) / DT_MOTION;
        d_i[1] = (d_i[0] - d_i_[0]) / DT_MOTION;

        s_i_[1] = (s_i_[0] - s_i__[0]) / DT_MOTION;
        d_i_[1] = (d_i_[0] - d_i__[0]) / DT_MOTION;

        s_i[2] = (s_i[1] - s_i_[1]) / DT_MOTION;
        d_i[2] = (d_i[1] - d_i_[1]) / DT_MOTION;

    }
}

double Motion::getPreviousPathTravelTime()
{
    return (_path_overlap < 3) ? 0 : ((N_POINTS_MOTION - _previous_path_s.size()) * DT_MOTION);
}

double Motion::getPreviousPathOverlapTime()
{
    return (_path_overlap < 3) ? 0 : (_path_overlap * DT_MOTION);
}



void Motion::generateMotion(Trajectory * trajectory, Track * track, int lane, double ref_vel)
{
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
        double next_wp_s = s_i[0]+i;
        double next_wp_d = 2+4*lane;

        vector<double> next_wp = track->getXY(next_wp_s, next_wp_d);
        ptss.push_back(next_wp_s);
        ptsd.push_back(next_wp_d);

        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }


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

    //print_vector(ptss, "M car ptss");
    //print_vector(ptsx, "M car ptsx");
    //print_vector(ptsy, "M car ptsy");

    tk::spline s;
    s.set_points(ptsx, ptsy);

    _next_s_vals.clear();
    _next_d_vals.clear();
    _next_x_vals.clear();
    _next_y_vals.clear();


    for(int i = 0; i < _path_overlap; i++)
    {
        _next_x_vals.push_back(_previous_path_x[i]);
        _next_y_vals.push_back(_previous_path_y[i]);
        _next_s_vals.push_back(_previous_path_s[i]);
        _next_d_vals.push_back(_previous_path_d[i]);
    }


    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;
    double N = (target_dist/(DT_MOTION*ref_vel));


    for(int i = 0; i < N_POINTS_MOTION - _path_overlap; i++) {
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        _next_x_vals.push_back(x_point);
        _next_y_vals.push_back(y_point);
    }

    //print_vector(_next_s_vals, "M next_s_vals-", 10);
    //print_vector(_next_x_vals, "M next_x_vals-", 10);
    //print_vector(_next_y_vals, "M next_y_vals-", 10);


    for(int i = _path_overlap; i < N_POINTS_MOTION; i++) {

        double x_point_map = ref_x + _next_x_vals[i]*cos(ref_yaw) - _next_y_vals[i]*sin(ref_yaw);
        double y_point_map = ref_y + _next_x_vals[i]*sin(ref_yaw) + _next_y_vals[i]*cos(ref_yaw);

        double theta = (i > 0) ? atan2(y_point_map - _next_y_vals[i-1], x_point_map - _next_x_vals[i-1]) : yaw_i; 
        vector<double> sd = track->getFrenet(x_point_map, y_point_map, theta);
        _next_s_vals.push_back(sd[0]);
        _next_d_vals.push_back(sd[1]);

        _next_x_vals[i] = x_point_map;
        _next_y_vals[i] = y_point_map;


    }

}
