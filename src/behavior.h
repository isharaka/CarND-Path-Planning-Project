
#ifndef _BEHAVIOR_
#define _BEHAVIOR_

#include <vector>
#include <string>
#include "track.h"
#include "car.h"

class Behavior
{
public:
    Behavior();

    struct target {
        int lane;
        double speed;
        bool too_close;
    };

    struct target generateBehavior(Car& ego, map<int, Car>& cars, Track * track, double planning_duration);

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

    enum state _state;

    vector<vector<Car>> _traffic;
    vector<vector<Car>> _traffic_predicted;

    vector<vector<double>> _kinematics;
    vector<vector<double>> _predicted_kinematics;

    void updateTraffic(Car& ego, map<int, Car>& cars, Track * track);
    bool carsInLane(int lane, bool predicted = true);
    bool carAheadInLane(int lane, Car& ego, Car& other, bool predicted = true);
    bool carBehindInLane(int lane, Car& ego, Car& other, bool predicted = true);
    double laneSpeed(int lane, Car& ego, bool predicted = true);

    void updateLaneKinematics(Car& ego, double duration, bool predicted = true);
    vector<double> getLaneKinematics(Car& ego, int lane, double duration, bool predicted = true);

    vector<Car> keepLaneTrajectory(enum state state, Car& ego, Track * track, double duration);
    vector<Car> prepLaneChangeTrajectory(enum state state, Car& ego, Track * track, double duration);
    vector<Car> laneChangeTrajectory(enum state state, Car& ego, Track * track, double duration);
    vector<Car> generateTrajectory(enum state state, Car& ego, Track * track, double duration);

    double getOuterLaneCost(enum state state, Car& ego, vector<Car>& trajectory, Track * track);
    double getEfficiencyCost(enum state state, Car& ego, vector<Car>& trajectory, Track * track);
    double getCost(enum state state, Car& ego, vector<Car>& trajectory, Track * track);

    vector<enum state> successorStates(Car& ego, Track * track);
    struct target chooseNextState(Car& ego, Track * track, double duration);

    static const string _state_names[NUM_STATES];
    static map<enum state, int> _lane_direction;
    static const double _front_buffer;
    static const double _rear_buffer;
};

#endif