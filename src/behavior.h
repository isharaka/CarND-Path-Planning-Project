#include <vector>
#include "prediction.h"
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

    struct target generateBehavior(Car& ego, map<int, Car>& cars, Map * track);


private:
    enum state {
        KEEP_LANE = 0,
        PREPARE_CHANGE_LANE_LEFT = 1,
        PREPARE_CHANGE_LANE_RIGHT = 2,
        CHANGE_LANE_LEFT = 3,
        CHANGE_LANE_RIGHT = 4,
    };

    struct target _target;
    enum state _state;
};