#ifndef _CAR_
#define _CAR_

#include <vector>

#include "map.h"

using namespace std;

class Car
{
    public:
        Car();
        Car(const Car& other);
        Car(int id, vector<double> s, vector<double> d, vector<double> s_predicted, vector<double> d_predicted);

        Car& operator=(const Car& other);

        int _id;
        vector<double> _s;
        vector<double> _d;
        vector<double> _s_predicted;
        vector<double> _d_predicted;
};

#endif
