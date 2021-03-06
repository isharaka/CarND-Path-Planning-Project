#include <vector>

#include "prediction.h"

void print_vector(vector<double>& vec, const string& name, int n=0);
void print_vector(vector<double>& vec, const string& name, int b, int e);

Car::Car()
{

}

Car::Car(int id, 
    vector<double> s, 
    vector<double> d, 
    double t_predicted,
    vector<double> s_predicted, 
    vector<double> d_predicted):
        _id(id), _s(s), _d(d), _t_predicted(t_predicted), _s_predicted(s_predicted), _d_predicted(d_predicted)
{

}

Car::Car(int id, 
    vector<double> s, 
    vector<double> d):
        _id(id), _s(s), _d(d), _t_predicted(0.0), _s_predicted(s), _d_predicted(d)
{

}

Car::Car(const Car& other)
{
    _id = other._id;
    _s = other._s;
    _d = other._d;
    _t_predicted = other._t_predicted;
    _s_predicted = other._s_predicted;
    _d_predicted = other._d_predicted;
}

Car& Car::operator=(const Car& other)
{
    if(&other == this)
        return *this;

    _id = other._id;
    _s = other._s;
    _d = other._d;
    _t_predicted = other._t_predicted;
    _s_predicted = other._s_predicted;
    _d_predicted = other._d_predicted;

    return *this;
}




