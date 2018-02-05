#include <vector>

#include "prediction.h"
#include "trajectory.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);


Prediction::Prediction()
{

}

map<int, const Car> Prediction::predict(Track * track, double duration, const Car& ego, vector<vector<double>>& sensor_fusion)
{
    map<int, const Car> cars;

    for (int i=0; i < sensor_fusion.size(); i++) {

        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];

        vector<double> dxdy = track->getDxDy(s, true, true);
        vector<double> sxsy = track->getSxSy(s, true);

        double s_dot = vx*sxsy[0] + vy*sxsy[1];
        double d_dot = vx*dxdy[0] + vy*dxdy[1];

        double s_prediction = s + s_dot * duration;
        double d_prediction = d + d_dot * duration;

        if (ego._s_predicted[0] > (s_prediction + 0.5*track->max_s))
            s_prediction += track->max_s;

        if (ego._s[0] > (s + 0.5*track->max_s))
            s += track->max_s;

        cars.insert(make_pair(sensor_fusion[i][0], Car(sensor_fusion[i][0], {s,s_dot,0}, {d,d_dot,0}, duration, {s_prediction,s_dot,0}, {d_prediction,d_dot,0})));

        // cout << "car id:" << cars[car._id]._id << " s dot:" << vx*sxsy[0] + vy*sxsy[1] <<endl;
        // print_vector(cars[car._id]._s_predicted, "car s");
        // print_vector(cars[car._id]._d_predicted, "car d");
    }

    return cars;
}

Car Prediction::predict(Track * track, double duration, vector<double> s, vector<double> d, Trajectory* trajectory, double t)
{
   return Car(-1, s, d, duration, trajectory->s(t), trajectory->d(t)); 
}

