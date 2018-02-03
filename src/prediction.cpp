#include <vector>

#include "prediction.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);


Prediction::Prediction()
{

}

/*vector<vector<vector<double>>> Prediction::predict(double duration)
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
}*/

map<int, Car> Prediction::predict(double duration, vector<vector<double>>& sensor_fusion)
{
    map<int, Car> cars;

    for (int i=0; i < sensor_fusion.size(); i++) {

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];

        vector<double> s_prediction(3);

        s_prediction[1] = sqrt(vx*vx + vy*vy);
        s_prediction[0] = s + s_prediction[1] * duration;

        vector<double> d_prediction(3);

        d_prediction[0] = d;

        Car car(sensor_fusion[i][0], {sensor_fusion[i][5],0,0}, {sensor_fusion[i][6],0,0}, s_prediction, d_prediction);
        cars[car._id] = car;

        //cout << "car id:" << cars[car._id]._id << " ";
        //print_vector(cars[car._id]._s_predicted, "car s");
    }

    return cars;
}

