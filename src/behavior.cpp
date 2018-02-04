#include "behavior.h"

#include <iostream>

using namespace std;
void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);

const double Behavior::speed_limit = 22.0;
const double Behavior::max_acceleration = 9.0;
const string Behavior::_state_names[NUM_STATES] = {"KL", "PCLL", "PCLR", "CLL", "CLR"};
map<enum Behavior::state, int> Behavior::_lane_direction = {{Behavior::KEEP_LANE, 0}, 
                                                        {Behavior::PREPARE_CHANGE_LANE_LEFT, -1}, 
                                                        {Behavior::CHANGE_LANE_LEFT, -1}, 
                                                        {Behavior::PREPARE_CHANGE_LANE_RIGHT, 1}, 
                                                        {Behavior::CHANGE_LANE_RIGHT, 1}};

Behavior::Behavior():_target{1,20.0}, _traffic(3), _traffic_predicted(3), _state(KEEP_LANE)
{}

struct compare_s
{
    inline bool operator() (const Car& car1, const Car& car2)
    {
        return (car1._s[0] > car2._s[0]);
    }
};  

struct ahead
{
    double _s;
    ahead(double s):_s(s) {}

    inline bool operator() (const Car& car)
    {
        return (car._s[0] > _s);
    }
}; 

struct compare_s_predicted
{
    inline bool operator() (const Car& car1, const Car& car2)
    {
        return (car1._s_predicted[0] > car2._s_predicted[0]);
    }
};  

struct ahead_predicted
{
    double _s_predicted;
    ahead_predicted(double s_predicted):_s_predicted(s_predicted) {}

    inline bool operator() (const Car& car)
    {
        return (car._s_predicted[0] > _s_predicted);
    }
}; 

void Behavior::updateTraffic(Car& ego, map<int, Car>& cars, Map * track)
{
    double ego_predicted_s = ego._s_predicted[0];
    double ego_s = ego._s[0];   

    for (int i = 0; i < _traffic.size(); ++i)
        _traffic[i].clear();   

    for (int i = 0; i < _traffic_predicted.size(); ++i)
        _traffic_predicted[i].clear(); 


    for (std::map<int,Car>::iterator it=cars.begin(); it!=cars.end(); ++it) {
        Car car = it->second;

        if (ego_predicted_s > (car._s_predicted[0] + 0.5*track->max_s))
            car._s_predicted[0] += track->max_s;

        if (ego_s > (car._s[0] + 0.5*track->max_s))
            car._s[0] += track->max_s;

        cars[it->first] = car;

        int lane = track->getLane(car._d[0]);

        // cout << "car id:" << car._id << " lane:" << lane << endl;
        // print_vector(car._s_predicted, "car s");
        // print_vector(car._d_predicted, "car d");

        if (lane >= 0 && lane < _traffic.size())
            _traffic[lane].push_back(car);

        int lane_predicted = track->getLane(car._d_predicted[0]);

        if (lane_predicted >= 0 && lane_predicted < _traffic_predicted.size())
            _traffic_predicted[lane].push_back(car);
    }  

    cout << "ego s:" << ego_s << " d:" << ego._d[0] << " lane:" << track->getLane(ego._d[0]) << endl;

    for(int i = 0; i < _traffic.size(); ++i) {
        std::sort(_traffic[i].begin(), _traffic[i].end(), compare_s());
        int n_cars_ahead = count_if (_traffic[i].begin(), _traffic[i].end(), ahead(ego_s));

        // cout << _traffic[i].size() << ' ' << n_cars_ahead << ' ';

        // for (int j = 0; j < _traffic[i].size(); ++j) {
        //     cout << ' ' << _traffic[i][j]._id << ':' << _traffic[i][j]._s_predicted[0];
        // }
        // cout << endl;

        if (n_cars_ahead > 1)
            _traffic[i].erase(_traffic[i].begin(), _traffic[i].begin() + n_cars_ahead - 1);

        n_cars_ahead = count_if (_traffic[i].begin(), _traffic[i].end(), ahead(ego_s));

        cout << _traffic[i].size() << ' ' << n_cars_ahead << ' ';

        for (int j = 0; j < _traffic[i].size(); ++j) {
            cout << ' ' << _traffic[i][j]._id << ':' << _traffic[i][j]._s[0] << "->" << _traffic[i][j]._s_predicted[0];
        }
        cout << endl;
    }

    cout << " ego_predicted_s:" << ego_predicted_s << " d:" << ego._d_predicted[0] << " lane:" << track->getLane(ego._d_predicted[0]) <<endl;

    for(int i = 0; i < _traffic_predicted.size(); ++i) {
        std::sort(_traffic_predicted[i].begin(), _traffic_predicted[i].end(), compare_s_predicted());
        int n_cars_ahead = count_if (_traffic_predicted[i].begin(), _traffic_predicted[i].end(), ahead_predicted(ego_predicted_s));

        // cout << _traffic[i].size() << ' ' << n_cars_ahead << ' ';

        // for (int j = 0; j < _traffic[i].size(); ++j) {
        //     cout << ' ' << _traffic[i][j]._id << ':' << _traffic[i][j]._s_predicted[0];
        // }
        // cout << endl;

        if (n_cars_ahead > 1)
            _traffic_predicted[i].erase(_traffic_predicted[i].begin(), _traffic_predicted[i].begin() + n_cars_ahead - 1);

        n_cars_ahead = count_if (_traffic_predicted[i].begin(), _traffic_predicted[i].end(), ahead_predicted(ego_predicted_s));

        cout << _traffic_predicted[i].size() << ' ' << n_cars_ahead << ' ';

        for (int j = 0; j < _traffic_predicted[i].size(); ++j) {
            cout << ' ' << _traffic_predicted[i][j]._id << ':' << _traffic_predicted[i][j]._s[0] << "->" << _traffic_predicted[i][j]._s_predicted[0];
        }
        cout << endl;
    }



}

bool Behavior::carsInLane(int lane, bool predicted)
{
    return predicted ? (_traffic_predicted[lane].size() > 0) : (_traffic[lane].size() > 0);
}

bool Behavior::carAheadInLane(int lane, Car& ego, Car& other, bool predicted)
{
    if (predicted) {
        if ((_traffic_predicted[lane].size() > 0) && (_traffic_predicted[lane][0]._s[0] > ego._s[0])) {
            other = _traffic_predicted[lane][0];
            return true;
        } else {
            return false;
        }
    } else {
        if ((_traffic[lane].size() > 0) && (_traffic[lane][0]._s[0] > ego._s[0])) {
            other = _traffic[lane][0];
            return true;
        } else {
            return false;
        }
    }
}

bool Behavior::carBehindInLane(int lane, Car& ego, Car& other, bool predicted)
{
    if (predicted) {
         if (_traffic_predicted[lane].size() > 1) {
            other = _traffic_predicted[lane][1];
            return true;
         } else if (_traffic_predicted[lane].size() == 1 && _traffic_predicted[lane][0]._s[0] < ego._s[0]) {
            other = _traffic_predicted[lane][0];
            return true;
         } else {
            return false;
         }
    } else {
         if (_traffic[lane].size() > 1) {
            other = _traffic[lane][1];
            return true;
         } else if (_traffic[lane].size() == 1 && _traffic[lane][0]._s[0] < ego._s[0]) {
            other = _traffic[lane][0];
            return true;
         } else {
            return false;
         }
    }
}

double Behavior::laneSpeed(int lane, Car& ego, bool predicted)
{
    #if 0
    if (predicted)
        return (_traffic_predicted[lane].size() > 0) ? min(_traffic_predicted[lane][0]._s_predicted[1], speed_limit) : speed_limit;
    else
        return (_traffic[lane].size() > 0) ? min(_traffic[lane][0]._s_predicted[1], speed_limit) : speed_limit;
    #else
    return getLaneKinematics(ego, lane, predicted)[1];
    #endif
}


vector<double> Behavior::getLaneKinematics(Car& ego, int lane, double duration, bool predicted) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = max_acceleration * duration + ego._s[1];
    double new_position;
    double new_velocity;
    double new_accel;

    if (carsInLane(lane, predicted)) { // there are cars in lane
        Car car_ahead;

        if (carAheadInLane(lane, ego, car_ahead, predicted)) { //cout << 'A';// there is a car ahead 
            Car car_behind;   

            if (carBehindInLane(lane, ego, car_behind, predicted)) { //cout << 'B';// there is a car behind          
                double velocity_in_front = car_ahead._s_predicted[1]; //must travel at the speed of traffic, regardless of preferred buffer
                new_velocity = min(min(velocity_in_front, max_velocity_accel_limit), speed_limit);
            } else { //cout << ' ';
                double max_velocity_in_front = 2 * (car_ahead._s_predicted[0] - ego._s[0] - 30) / duration - car_ahead._s_predicted[1];
                new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), speed_limit);
            }
        } else { //cout << "  ";
            new_velocity = min(max_velocity_accel_limit, speed_limit);
        }

    } else { //cout << "  ";
        new_velocity = min(max_velocity_accel_limit, speed_limit);
    }

    //cout << ' ';
    
    new_accel = (new_velocity - ego._s_predicted[1]) / duration; //Equation: (v_1 - v_0)/t = acceleration
    new_position = ego._s_predicted[0] + new_velocity * duration + new_accel * duration * duration / 2.0;
    return {new_position, new_velocity, new_accel};    
}

vector<Car> Behavior::keepLaneTrajectory(enum state state, Car& ego, Map * track, double duration) {
    /*
    Generate a keep lane trajectory.
    */
    int current_lane = track->getLane(ego._d[0]);
    vector<double> current_lane_kinematics = getLaneKinematics(ego, current_lane, duration);
    return {Car(-100, ego._s, ego._d), Car(-200, current_lane_kinematics, ego._d)};
}

vector<Car> Behavior::prepLaneChangeTrajectory(enum state state, Car& ego, Map * track, double duration) {
    /*
    Generate a trajectory preparing for a lane change.
    */

    int current_lane = track->getLane(ego._d[0]);
    int new_lane = current_lane + _lane_direction[state];

    vector<double> best_kinematics;
    vector<double> current_lane_kinematics = getLaneKinematics(ego, current_lane, duration);

    Car car_behind;

    if (carBehindInLane(current_lane, ego, car_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        best_kinematics = current_lane_kinematics;
    } else {
        
        vector<double> new_lane_kinematics = getLaneKinematics(ego, new_lane, duration);
        //Choose kinematics with lowest velocity.
        if (new_lane_kinematics[1] < current_lane_kinematics[1]) {
            best_kinematics = new_lane_kinematics;
        } else {
            best_kinematics = current_lane_kinematics;
        }
    }

    return {Car(-100, ego._s, ego._d), Car(-200, best_kinematics, ego._d)};
}

vector<Car> Behavior::laneChangeTrajectory(enum state state, Car& ego, Map * track, double duration)
{
    /*
    Generate a lane change trajectory.
    */    
    int current_lane = track->getLane(ego._d[0]);
    int new_lane = min(track->rightmost_lane, max(track->leftmost_lane, current_lane + _lane_direction[state]));

    vector<double> new_lane_kinematics = getLaneKinematics(ego, new_lane, duration);
    vector<double> current_lane_kinematics = getLaneKinematics(ego, current_lane, duration);

    bool lane_change_safe = true;

    //Check if a lane change is possible (check if another vehicle occupies that spot).
    if (carsInLane(new_lane)) {

        Car car_ahead;
        if (carAheadInLane(new_lane, ego, car_ahead)) {
            if (car_ahead._s_predicted[0] < ego._s_predicted[0] + 30)
                lane_change_safe = false;
        } 

        Car car_behind;
        if (carBehindInLane(new_lane, ego, car_behind)) {
            if (car_behind._s_predicted[0] + 30 > ego._s_predicted[0])
                lane_change_safe = false;
        } 
    }

    if (lane_change_safe)
        return {Car(-100, ego._s, ego._d), Car(-200, new_lane_kinematics, {track->getD(new_lane), 0, 0})};
    else
        return {Car(-100, ego._s, ego._d), Car(-200, current_lane_kinematics, ego._d)};
}


vector<Car> Behavior::generateTrajectory(enum state state, Car& ego, Map * track, double duration) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */

    vector<Car> trajectory;

    switch(state) {

    case PREPARE_CHANGE_LANE_LEFT: 
    case PREPARE_CHANGE_LANE_RIGHT:
        return prepLaneChangeTrajectory(state, ego, track, duration);
        break;

    case CHANGE_LANE_LEFT:  
    case CHANGE_LANE_RIGHT:  
        return laneChangeTrajectory(state, ego, track, duration);
        break; 

    case KEEP_LANE: 
    default:
        return keepLaneTrajectory(state, ego, track, duration);
        break;  
    }

    return trajectory;
}

double Behavior::getEfficiencyCost(enum state state, Car& ego, vector<Car>& trajectory, Map * track)
{
    double efficiency_cost = 1.0;

    if (trajectory.size() == 0)
        return 1.0;

    Car last = trajectory[trajectory.size() - 1];

    int final_lane = track->getLane(last._d[0]);
    int intended_lane;

    switch(state) {
    case PREPARE_CHANGE_LANE_LEFT: 
        intended_lane = final_lane - 1;
        break;
    case PREPARE_CHANGE_LANE_RIGHT: 
        intended_lane = final_lane + 1;
        break;
    default:
        intended_lane = final_lane;
        break;        
    }

    double final_speed = laneSpeed(final_lane, ego);
    double intended_speed = laneSpeed(intended_lane, ego);

    efficiency_cost = (2.0*speed_limit - final_speed - intended_speed) / (2.0*speed_limit);

    return efficiency_cost;
}

double Behavior::getCost(enum state state, Car& ego, vector<Car>& trajectory, Map * track)
{
    double cost = 1.0;

    if (trajectory.size() == 0)
        return cost;

    double efficiency_cost = getEfficiencyCost(state, ego, trajectory, track);

    return efficiency_cost;
}



vector<enum Behavior::state> Behavior::successorStates(Car& ego, Map * track) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    int ego_lane = track->getLane(ego._d_predicted[0]); 

    vector<enum state> states;
    states.push_back(KEEP_LANE);

    switch(_state) {
    case KEEP_LANE:  
        if (ego_lane != track->leftmost_lane)
            states.push_back(PREPARE_CHANGE_LANE_LEFT);
        if (ego_lane != track->rightmost_lane)
            states.push_back(PREPARE_CHANGE_LANE_RIGHT);
        break;

    case PREPARE_CHANGE_LANE_LEFT:  
        states.push_back(PREPARE_CHANGE_LANE_LEFT);
        states.push_back(CHANGE_LANE_LEFT);
        break;
    case PREPARE_CHANGE_LANE_RIGHT:  
        states.push_back(PREPARE_CHANGE_LANE_RIGHT);
        states.push_back(CHANGE_LANE_RIGHT);
        break;
    case CHANGE_LANE_LEFT:  
        states.push_back(CHANGE_LANE_LEFT);
        break;
    case CHANGE_LANE_RIGHT: 
        states.push_back(CHANGE_LANE_RIGHT); 
        break;      
    default:
        break;  
    }

    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}


struct Behavior::target Behavior::chooseNextState(Car& ego, Map * track, double duration) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    

    vector<enum state> states = successorStates(ego, track);

    //cout << "num states:" << states.size() << ' ';

    //for (int i = 0; i < states.size(); ++i)
    //    cout << _state_names[states[i]] << ' ';

    cout << endl;

    double best_cost = 1.0;
    enum state best_state;

    map<enum state, vector<Car>> trajectories;


    for (int i = 0; i < states.size(); ++i) { 
        cout << "!";       

        vector<Car> trajectory = generateTrajectory(states[i], ego, track, duration);
        trajectories[states[i]] = trajectory;

        double cost = getCost(states[i], ego, trajectory, track);

        if (cost < best_cost) {
            best_cost = cost;
            best_state = states[i];
        }

        cout << _state_names[states[i]] << "[" << cost << "] ";

        if (trajectory.size() > 1) {
            cout << trajectory[0]._s[0] << ':' << trajectory[0]._s[1] << '(' << track->getLane(trajectory[0]._d[0]) << ") => " 
            << trajectory[1]._s[0] << ':' << trajectory[1]._s[1]<< '(' << track->getLane(trajectory[1]._d[0]) << ')';   
        }
        cout << "*";
        cout << endl;
    }

    if (best_cost < 1)
        _state = best_state;
    else
        _state = KEEP_LANE;

    //_state = KEEP_LANE;

    struct target intended_target = {track->getLane(trajectories[_state][1]._d[0]), trajectories[_state][1]._s[1]};
    cout << "_state: " << _state_names[_state] << " speed:" << intended_target.speed << " lane:" << intended_target.lane << endl;

    return intended_target;
}



struct Behavior::target Behavior::generateBehavior(Car& ego, map<int, Car>& cars, Map * track, double planning_duration)
{
    bool too_close = false;  
    int ego_lane = track->getLane(ego._d_predicted[0]); 
    double ego_predicted_s = ego._s_predicted[0];
    double ego_s = ego._s[0];

    cout << "_state: " << _state_names[_state] << " speed:" << _target.speed << " lane:" << _target.lane << endl;

    updateTraffic(ego, cars, track);

    for (int i = 0; i < 3; ++i) {
        vector<double> lane_s = getLaneKinematics(ego, i, planning_duration);
        cout << i << ' ';
        if (_traffic[i].size() > 0 )
            cout << _traffic[i][0]._s_predicted[1] << ' ';
        print_vector(lane_s, "lane_s");
    }

    struct target intended_behavior = chooseNextState(ego, track, planning_duration);

    double intended_speed;
    

    if (_traffic_predicted[ego_lane].size() > 0) {
        if ((ego_s < _traffic_predicted[ego_lane][0]._s[0]) && (_traffic_predicted[ego_lane][0]._s_predicted[0] - ego_predicted_s < 30)) {
            too_close = true;
            cout << "C";
            intended_speed = _traffic_predicted[ego_lane][0]._s_predicted[1];
        } else {
            intended_speed = intended_behavior.speed;
        }
    } else {
        intended_speed = intended_behavior.speed;
    }

    _target.lane = intended_behavior.lane;


    if (too_close) {
        if (_target.speed > 5)
        _target.speed -= 1;
    } else if (_target.speed < intended_speed) {
        _target.speed += 1;
    }

    return _target;
}
