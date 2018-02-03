#ifndef _PREDICTION_
#define _PREDICTION_

#include <vector>

#include "map.h"
#include "car.h"
#include "trajectory.h"

using namespace std;

class Prediction
{
public:
    Prediction();

    map<int, Car> predict(double duration, vector<vector<double>>& sensor_fusion);
    Car predict(double duration, vector<double> s, vector<double> d, Trajectory * trajectory);

private:  
};

#endif