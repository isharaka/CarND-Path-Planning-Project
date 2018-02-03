#include "behavior.h"

Behavior::Behavior():_target{1,20.0}
{}


struct Behavior::target Behavior::generateBehavior(Car& ego, map<int, Car>& cars, Map * track)
{
    bool too_close = false;  
    int lane = track->getLane(ego._d_predicted[0]); 
    double ego_predicted_s = ego._s_predicted[0];
    double ego_s = ego._s[0];     

    for (std::map<int,Car>::iterator it=cars.begin(); it!=cars.end(); ++it) {
        Car car = it->second;

        if (ego_predicted_s > (car._s_predicted[0] + 0.5*track->max_s))
            car._s_predicted[0] += track->max_s;

        if (ego_s > (car._s[0] + 0.5*track->max_s))
            car._s[0] += track->max_s;

        cars[it->first] = car;
    }  

    for (std::map<int,Car>::iterator it=cars.begin(); it!=cars.end(); ++it) {
        Car car = it->second;

        if (car._d_predicted[0] < track->getD(lane) + 2 && car._d_predicted[0] > track->getD(lane) - 2) {
            double car_predicted_s = car._s_predicted[0];

            if (car_predicted_s > ego_predicted_s && (car_predicted_s- ego_predicted_s) < 30) {
                too_close = true;
            }
        }
    }

    if (too_close) {
        if (_target.speed > 5)
        _target.speed -= 1;
    } else if (_target.speed < 20) {
        _target.speed += 1;
    }

    return _target;
}
