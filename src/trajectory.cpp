#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "trajectory.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

const double Trajectory::time_horizon = 3.0;

Trajectory::Trajectory()
{

}

vector<double> Trajectory::s(double t)
{
    vector<double> s_dot_coeff = polyderiv(_s_coeff);

    return { polyeval(_s_coeff, t), polyeval(s_dot_coeff, t), 0 };
}

vector<double> Trajectory::d(double t)
{
    return {polyeval(_d_coeff, t), 0, 0};
}

void Trajectory::generateTrajectory(vector<double>& s_i, vector<double>& d_i, struct Behavior::target& target_behavior, Map * track)
{
#if 0
        vector<double> s_f = { s_i[0] + target_behavior.speed * time_horizon, target_behavior.speed, 0};
        vector<double> d_f = { track->getD(target_behavior.lane), 0, 0};

        generateCVTrajectory(s_i, d_i, s_f, d_f, time_horizon);
#else
        vector<double> s_f = { s_i[0] + (s_i[1] + target_behavior.speed) * time_horizon / 2, target_behavior.speed, 0};
        vector<double> d_f = { track->getD(target_behavior.lane), 0, 0};

        print_vector(s_i, "s_i");
        print_vector(s_f, "s_f");
        print_vector(d_i, "d_i");
        print_vector(d_f, "d_f");

        //print_vector(x_i, "x_i");
        //print_vector(y_i, "y_i");

        generateJMTrajectory(s_i, d_i, s_f, d_f, time_horizon);

#endif   
}


void Trajectory::generateCVTrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration)
{
    _s_coeff.clear();
    _d_coeff.clear();

    _s_coeff.push_back(s_i[0]);
    _s_coeff.push_back((s_f[0] - s_i[0])/duration);

    _d_coeff.push_back(d_i[0]);
    _d_coeff.push_back((d_f[0] - d_i[0])/duration);

    cout << "s_i:" << s_i[0] << " s_f:" << s_f[0] << " dur:" << duration 
    << " c0:" << _s_coeff[0] << " c1:" << _s_coeff[1] << endl;
    cout << "d_i:" << d_i[0] << " d_f:" << d_f[0] << " dur:" << duration 
    << " c0:" << _d_coeff[0] << " c1:" << _d_coeff[1] << endl;
}

void Trajectory::generateCATrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration)
{
    _s_coeff.clear();
    _d_coeff.clear();

    _s_coeff.push_back(s_i[0]);
    _s_coeff.push_back(s_i[1]);
    _s_coeff.push_back((s_f[1] - s_i[1])/(2*duration));

    _d_coeff.push_back(d_i[0]);
    _d_coeff.push_back(d_i[1]);
    _d_coeff.push_back((d_f[1] - d_i[1])/(2*duration));

    cout << "s_i:" << s_i[0] << " s_f:" << s_f[0]
    << " s_i':" << s_i[1] << " s_f':" << s_f[1] << " dur:" << duration 
    << " c0:" << _s_coeff[0] << " c1:" << _s_coeff[1] << " c2:" << _s_coeff[2] << endl;
    cout << "d_i:" << d_i[0] << " d_f:" << d_f[0]
    << " d_i':" << d_i[1] << " d_f':" << d_f[1] << " dur:" << duration 
    << " c0:" << _d_coeff[0] << " c1:" << _d_coeff[1] << " c2:" << _d_coeff[2] << endl;
}

void Trajectory::generateJMTrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration)
{
    _s_coeff.clear();
    _d_coeff.clear();

    _s_coeff = JMT(s_i, s_f, duration);
    _d_coeff = JMT(d_i, d_f, duration);
}


double Trajectory::polyeval(vector<double>& c, double x)
{
    double result = 0.0;
    for (int i = 0; i < c.size(); ++i)
        result += c[i]*pow(x,i);
}

vector<double> Trajectory::polyderiv(vector<double>& c)
{
    vector<double> derivative;

    for (int i = 1; i < c.size(); ++i)
        derivative.push_back(i*c[i]);

    return derivative;
}

double Trajectory::timeToDestination(double s)
{
    if (_s_coeff.size() > 2) {
        double a = _s_coeff[2];
        double b = _s_coeff[1];
        double c = _s_coeff[0] - s;

        double d = b*b - 4*a*c;

        if (d > 0 && a != 0) {
            d = sqrt(d);
            return std::max(d-b,d+b) / (2*a);
        } else {
            return -1;
        }

    } else if (_s_coeff.size() > 1) {
        double v = _s_coeff[1];
        if (v != 0)
            return (s - _s_coeff[0])/v;

    } else {
        return -1;
    }
}

vector<double> Trajectory::JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    MatrixXd a(3,3);
    double T2 =  T*T, 
           T3 = T2*T, 
           T4 = T3*T,
           T5 = T4*T;
    a <<  T3,    T4,    T5, 
        3*T2,  4*T3,  5*T4, 
         6*T, 12*T2, 20*T3;
    MatrixXd aInv = a.inverse();
    
    VectorXd b(3);
    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
         end[1] - (           start[1]   +     start[2]*T),
         end[2] - (                            start[2]);
    VectorXd alpha = aInv * b;

    
    vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};

    //print_vector(output, "jmt");
    return output;
}