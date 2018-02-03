#include <vector>

#include "map.h"

using namespace std;

class Car
{
    public:
        Car();
        Car(const Car& other);
        Car(int id, vector<double> s, vector<double> d, vector<double> s_predicted, vector<double> d_predicted);

        Car& operator=(Car& other);

        int _id;
        vector<double> _s;
        vector<double> _d;
        vector<double> _s_predicted;
        vector<double> _d_predicted;
};

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

    //vector<vector<vector<double>>> predict(double duration);
    map<int, Car> predict(double duration);


private:
    vector<vector<double>> _sensor_fusion;     
};