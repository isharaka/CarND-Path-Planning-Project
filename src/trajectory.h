#include <vector>

using namespace std;

class Trajectory
{
public:
    Trajectory();

    void generateCVTrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration);
    void generateCATrajectory(vector<double>& s_i, vector<double>& d_i, vector<double>& s_f, vector<double>& d_f, double duration);

    double s(double t);
    double d(double t);

private:
    vector<double> _s_coeff;
    vector<double> _d_coeff;

    vector<double> JMT(vector<double> start, vector<double> end, double T);
    double polyeval(vector<double>& c, double x);

};