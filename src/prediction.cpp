#include <vector>

#include "prediction.h"


Prediction::Prediction()
{

}

void Prediction::telemetry(
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
    )
{
    _sensor_fusion = sensor_fusion;
}

vector<vector<vector<double>>> Prediction::predict(double duration)
{
    _s_predictions.clear();
    _d_predictions.clear();

    vector<vector<vector<double>>> predictions;

    for (int i=0; i < _sensor_fusion.size(); i++) {

        double vx = _sensor_fusion[i][3];
        double vy = _sensor_fusion[i][4];
        double s = _sensor_fusion[i][5];
        double d = _sensor_fusion[i][6];

        vector<double> s_prediction(3);

        s_prediction[1] = sqrt(vx*vx + vy*vy);
        s_prediction[0] = s + s_prediction[1] * duration;

        _s_predictions.push_back(s_prediction);

        vector<double> d_prediction(3);

        d_prediction[0] = d;

        _d_predictions.push_back(d_prediction);

        vector<vector<double>> prediction;

        prediction.push_back(s_prediction);
        prediction.push_back(d_prediction);

        predictions.push_back(prediction);
    }

    return predictions;
}

