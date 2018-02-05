#ifndef _TRAJECTORY_
#define _TRAJECTORY_

#include <vector>
#include "behavior.h"
#include "map.h"

using namespace std;

class Trajectory
{
public:
    Trajectory();

    static const double time_horizon;

    void generateTrajectory(vector<double>& s_i, vector<double>& d_i, struct Behavior::target& intended_behavior, Map * track);


    vector<double> s(double t);
    vector<double> d(double t);

    double timeToDestination(double s);

private:
    vector<double> _s_coeff;
    vector<double> _d_coeff;

    struct Behavior::target _target_behavior;

    void generateCVTrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration);
    void generateCATrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration);
    void generateJMTrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration);

    vector<double> JMT(vector<double> start, vector<double> end, double T);

    double polyeval(vector<double>& c, double x);
    vector<double> polyderiv(vector<double>& c);

};

#endif