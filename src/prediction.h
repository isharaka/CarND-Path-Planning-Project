#ifndef _PREDICTION_
#define _PREDICTION_

#include <vector>

#include "map.h"
#include "car.h"

using namespace std;

class Prediction
{
public:
    Prediction();

    //vector<vector<vector<double>>> predict(double duration);
    map<int, Car> predict(double duration, vector<vector<double>>& sensor_fusion);


private:
    vector<vector<double>> _sensor_fusion;     
};

#endif