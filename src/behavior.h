
#ifndef _BEHAVIOR_
#define _BEHAVIOR_

#include <vector>
#include <string>
#include "map.h"
#include "car.h"

class Behavior
{
public:
    Behavior();

    struct target {
        int lane;
        double speed;
    };

    struct target generateBehavior(Car& ego, map<int, Car>& cars, Map * track, double planning_duration);

    static const double speed_limit;
    static const double max_acceleration;


private:
    enum state {
        KEEP_LANE = 0,
        PREPARE_CHANGE_LANE_LEFT = 1,
        PREPARE_CHANGE_LANE_RIGHT = 2,
        CHANGE_LANE_LEFT = 3,
        CHANGE_LANE_RIGHT = 4,

        NUM_STATES
    };

    struct target _target;
    enum state _state;

    vector<vector<Car>> _traffic;
    vector<vector<Car>> _traffic_predicted;

    void updateTraffic(Car& ego, map<int, Car>& cars, Map * track);
    bool carsInLane(int lane, bool predicted = true);
    bool carAheadInLane(int lane, Car& ego, bool predicted = true);
    bool carBehindInLane(int lane, Car& ego, bool predicted = true);


    vector<double> getLaneKinematics(Car& ego, int lane, double duration, bool predicted = true);

    vector<Car> keepLaneTrajectory(Car& ego, Map * track, double duration);
    vector<Car> generateTrajectory(enum state state, Car& ego, Map * track, double duration);

    vector<enum state> successorStates(Car& ego, Map * track);
    struct target chooseNextState(Car& ego, Map * track, double duration);

    static const string _state_names[NUM_STATES];
};

#endif