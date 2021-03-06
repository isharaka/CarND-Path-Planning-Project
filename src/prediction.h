#ifndef _PREDICTION_
#define _PREDICTION_

#include <vector>

#include "track.h"
#include "car.h"
#include "trajectory.h"

using namespace std;

class Prediction
{
public:
    Prediction();

    map<int, const Car> predict(Track * track, double duration, const Car& ego, vector<vector<double>>& sensor_fusion);
    Car predict(Track * track, double duration, vector<double> s, vector<double> d, Trajectory * trajectory, double t);

private:  
};

#endif