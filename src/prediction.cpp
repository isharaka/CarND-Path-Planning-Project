#include <vector>

#include "prediction.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);


Prediction::Prediction()
{

}

map<int, Car> Prediction::predict(Map * track, double duration, vector<vector<double>>& sensor_fusion)
{
    map<int, Car> cars;

    for (int i=0; i < sensor_fusion.size(); i++) {

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];

        vector<double> dxdy = track->getDxDy(s, true, true);
        vector<double> sxsy = track->getSxSy(s, true);

        vector<double> s_prediction(3);

        s_prediction[1] = vx*sxsy[0] + vy*sxsy[1]; //sqrt(vx*vx + vy*vy);
        s_prediction[0] = s + s_prediction[1] * duration;

        vector<double> d_prediction(3);

        d_prediction[0] = d;
        d_prediction[1] = vx*dxdy[0] + vy*dxdy[1];

        Car car(sensor_fusion[i][0], {sensor_fusion[i][5],0,0}, {sensor_fusion[i][6],0,0}, s_prediction, d_prediction);
        cars[car._id] = car;

        // cout << "car id:" << cars[car._id]._id << " s dot:" << vx*sxsy[0] + vy*sxsy[1] <<endl;
        // print_vector(cars[car._id]._s_predicted, "car s");
        // print_vector(cars[car._id]._d_predicted, "car d");
    }

    return cars;
}

Car Prediction::predict(Map * track, double duration, vector<double> s, vector<double> d, Trajectory* trajectory)
{
   return Car(-1, s, d, trajectory->s(duration), trajectory->d(duration)); 
}

