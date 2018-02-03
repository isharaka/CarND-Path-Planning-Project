#include "behavior.h"

#include <iostream>

using namespace std;
void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);

const double Behavior::speed_limit = 20.0;
const double Behavior::max_acceleration = 9.0;

Behavior::Behavior():_target{1,20.0}, _traffic(3)
{}

struct less_than_key
{
    inline bool operator() (const Car& car1, const Car& car2)
    {
        return (car1._s_predicted[0] > car2._s_predicted[0]);
    }
};  

struct ahead
{
    double _s;

    ahead(double s):_s(s) {}

    inline bool operator() (const Car& car)
    {
        return (car._s_predicted[0] > _s);
    }
}; 

void Behavior::updateTraffic(Car& ego, map<int, Car>& cars, Map * track)
{
    double ego_predicted_s = ego._s_predicted[0];
    double ego_s = ego._s[0];   

    for (int i = 0; i < _traffic.size(); ++i)
        _traffic[i].clear(); 


    for (std::map<int,Car>::iterator it=cars.begin(); it!=cars.end(); ++it) {
        Car car = it->second;

        if (ego_predicted_s > (car._s_predicted[0] + 0.5*track->max_s))
            car._s_predicted[0] += track->max_s;

        if (ego_s > (car._s[0] + 0.5*track->max_s))
            car._s[0] += track->max_s;

        cars[it->first] = car;

        int lane = track->getLane(car._d_predicted[0]);

        // cout << "car id:" << car._id << " lane:" << lane << endl;
        // print_vector(car._s_predicted, "car s");
        // print_vector(car._d_predicted, "car d");

        if (lane >= 0 && lane < _traffic.size())
            _traffic[lane].push_back(car);
    }  

    cout << "ego s " << ego_predicted_s << endl;

    for(int i = 0; i < _traffic.size(); ++i) {
        std::sort(_traffic[i].begin(), _traffic[i].end(), less_than_key());
        int n_cars_ahead = count_if (_traffic[i].begin(), _traffic[i].end(), ahead(ego_predicted_s));

        // cout << _traffic[i].size() << ' ' << n_cars_ahead << ' ';

        // for (int j = 0; j < _traffic[i].size(); ++j) {
        //     cout << ' ' << _traffic[i][j]._id << ':' << _traffic[i][j]._s_predicted[0];
        // }
        // cout << endl;

        if (n_cars_ahead > 1)
            _traffic[i].erase(_traffic[i].begin(), _traffic[i].begin() + n_cars_ahead - 1);

        n_cars_ahead = count_if (_traffic[i].begin(), _traffic[i].end(), ahead(ego_predicted_s));

        cout << _traffic[i].size() << ' ' << n_cars_ahead << ' ';

        for (int j = 0; j < _traffic[i].size(); ++j) {
            cout << ' ' << _traffic[i][j]._id << ':' << _traffic[i][j]._s_predicted[0];
        }
        cout << endl;
    }



}

vector<double> Behavior::getLaneKinematics(Car& ego, int lane, double duration) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = max_acceleration * duration + ego._s_predicted[1];
    double new_position;
    double new_velocity;
    double new_accel;

    if (_traffic[lane].size() > 0) { // there are cars in lane

        if (_traffic[lane][0]._s_predicted[0] > ego._s_predicted[0]) { // there is a car ahead
            Car car_ahead = _traffic[lane][0];
                cout << 'A';

            if (_traffic[lane].size() > 1) { // there is a car behind
                cout << 'B';
                new_velocity = car_ahead._s_predicted[1]; //must travel at the speed of traffic, regardless of preferred buffer
            } else {
                double max_velocity_in_front = 2 * (car_ahead._s_predicted[0] - ego._s_predicted[0] - 30) / duration - car_ahead._s_predicted[1];
                new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), speed_limit);
            }
        } else {
            new_velocity = min(max_velocity_accel_limit, speed_limit);
        }

    } else {
        new_velocity = min(max_velocity_accel_limit, speed_limit);
    }
    
    new_accel = (new_velocity - ego._s_predicted[1]) / duration; //Equation: (v_1 - v_0)/t = acceleration
    new_position = ego._s_predicted[0] + new_velocity * duration + new_accel * duration * duration / 2.0;
    return {new_position, new_velocity, new_accel};    
}


struct Behavior::target Behavior::generateBehavior(Car& ego, map<int, Car>& cars, Map * track, double planning_duration)
{
    bool too_close = false;  
    int ego_lane = track->getLane(ego._d_predicted[0]); 
    double ego_predicted_s = ego._s_predicted[0];

    updateTraffic(ego, cars, track);

    for (int i = 0; i < 3; ++i) {
        vector<double> lane_s = getLaneKinematics(ego, i, planning_duration);
        cout << i << ' ';
        if (_traffic[i].size() > 0 )
            cout << _traffic[i][0]._s_predicted[1] << ' ';
        print_vector(lane_s, "lane_s");
    }

    double intended_speed;
    

    if (_traffic[ego_lane].size() > 0) {
        if (_traffic[ego_lane][0]._s_predicted[0] - ego_predicted_s < 30) {
            too_close = true;
            cout << "too close " << endl;
            intended_speed = _traffic[ego_lane][0]._s_predicted[1];
        } else {
            intended_speed = 20.0;
        }
    } else {
        intended_speed = 20.0;
    }


    if (too_close) {
        if (_target.speed > 5)
        _target.speed -= 1;
    } else if (_target.speed < intended_speed) {
        _target.speed += 1;
    }

    return _target;
}
