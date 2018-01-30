#include <vector>

#include "trajectory.h"
#include "map.h"

using namespace std;

class Motion
{
public:
    Motion();

    void telemetry(
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
        );

    void generateMotion(Trajectory * trajectory, Map * track);
    void getMotion(vector<double>& next_x_vals, vector<double>& next_y_vals);

    vector<double> getInitS() { return s_i;}
    vector<double> getInitD() { return d_i;}
    vector<double> getInitX() { return x_i;}
    vector<double> getInitY() { return y_i;}
    double getInitYaw() { return yaw_i;}

private:
    double calculateDerivatives();

        double _car_x;
        double _car_y;
        double _car_s;
        double _car_d;
        double _car_yaw;
        double _car_speed;
        vector<double> _previous_path_x;
        vector<double> _previous_path_y;
        vector<double> _previous_path_s;
        vector<double> _previous_path_d;
        double _end_path_s;
        double _end_path_d;

        vector<double> _next_x_vals;
        vector<double> _next_y_vals;
        vector<double> _next_s_vals;
        vector<double> _next_d_vals;

        vector<double> s_i;
        vector<double> d_i;
        vector<double> x_i;
        vector<double> y_i;
        double yaw_i;

        vector<double> s_i_;
        vector<double> d_i_;
        vector<double> x_i_;
        vector<double> y_i_;

        Trajectory * _trajectory;


        int _path_overlap;
    
};