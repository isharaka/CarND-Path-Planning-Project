#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Dense"
#include "trajectory.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

Trajectory::Trajectory()
{

}

double Trajectory::s(double t)
{
    return polyeval(_s_coeff, t);
}

double Trajectory::d(double t)
{
    return polyeval(_d_coeff, t);
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
    _s_coeff.push_back((s_f[1] - s_i[1])/(1*duration));

    _d_coeff.push_back(d_i[0]);
    //_d_coeff.push_back(d_i[1]);
    //_d_coeff.push_back((d_f[1] - d_i[1])/(1*duration));

    cout << "s_i:" << s_i[0] << " s_f:" << s_f[0]
    << " s_i':" << s_i[1] << " s_f':" << s_f[1] << " dur:" << duration 
    << " c0:" << _s_coeff[0] << " c1:" << _s_coeff[1] << " c2:" << _s_coeff[2] << endl;
    cout << "d_i:" << d_i[0] << " d_f:" << d_f[0]
    << " d_i':" << d_i[1] << " d_f':" << d_f[1] << " dur:" << duration 
    << " c0:" << _d_coeff[0] << " c1:" << _d_coeff[1] << " c2:" << _d_coeff[2] << endl;
}

double Trajectory::polyeval(vector<double>& c, double x)
{
    double result = 0.0;
    for (int i = 0; i < c.size(); ++i)
        result += c[i]*pow(x,i);
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
    return output;
}