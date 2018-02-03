#include <vector>
#include "prediction.h"
#include "map.h"

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
    struct target _target;
};