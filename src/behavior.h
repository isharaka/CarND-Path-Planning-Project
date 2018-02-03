#include <vector>
#include "prediction.h"
#include "map.h"

class Behavior
{
public:
    Behavior();

    struct target {
        int lane;
        int speed;
    };

    struct target generateBehavior();


private:
};