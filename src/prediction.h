#include <vector>

#include "map.h"

using namespace std;

class Prediction
{
public:
    Prediction();

    void telemetry(
            Map * track,
            double car_x,
            double car_y,
            double car_s,
            double car_d,
            double car_yaw,
            double car_speed,
            vector<double> previous_path_x,
            vector<double> previous_path_y,
            double end_path_s,
            double end_path_d,
            vector<vector<double>> sensor_fusion
        );

    vector<vector<double>> getSPredictions() { return _s_predictions;}
    vector<vector<double>> getDPredictions() { return _d_predictions;}

    vector<vector<vector<double>>> predict(double duration);


private:
    vector<vector<double>> _s_predictions;
    vector<vector<double>> _d_predictions;
    vector<vector<double>> _sensor_fusion;     
};