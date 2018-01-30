#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"

#include "motion.h"


#define N_POINTS_MOTION  50
#define PREVIOUS_PATH_OVERLAP  25
#define DT_MOTION  0.02

Motion::Motion():
    s_i(3), d_i(3), x_i(3), y_i(3), s_i_(3), d_i_(3), x_i_(3), y_i_(3)
{

}

void Motion::telemetry(
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
      //_previous_path_s.push_back(_next_s_vals[i]);
      //_previous_path_d.push_back(_next_d_vals[i]);
    }

    int prev_size = previous_path_x.size();

    if (prev_size == 0) {
        _end_path_s = car_s;
        _end_path_d = car_d;
    }

    _path_overlap = std::min((int)PREVIOUS_PATH_OVERLAP, prev_size);

    //calculateDerivatives();

}

void Motion::generateMotion(Trajectory * trajectory, Map * track)
{

}

void Motion::getMotion(vector<double>& next_x_vals, vector<double>& next_y_vals)
{
    
}


double Motion::calculateDerivatives()
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


        s_i[1] = (s_i[0] - s_i_[0]) / DT_MOTION;
        d_i[1] = (d_i[0] - d_i_[0]) / DT_MOTION;

        s_i_[1] = (s_i_[0] - s_i__[0]) / DT_MOTION;
        d_i_[1] = (d_i_[0] - d_i__[0]) / DT_MOTION;

        s_i[2] = (s_i[1] - s_i_[1]) / DT_MOTION;
        d_i[2] = (d_i[1] - d_i_[1]) / DT_MOTION;

    }
}
