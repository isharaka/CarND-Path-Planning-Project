#include <vector>

#include "prediction.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);


Prediction::Prediction()
{

}

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

Car Prediction::predict(double duration, vector<double> s, vector<double> d, Trajectory* trajectory)
{
   return Car(-1, s, d, trajectory->s(duration), trajectory->d(duration)); 
}

