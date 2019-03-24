#include <stdlib.h>
#include <math.h>
#include <angles/angles.h>

class Point_Controller
{
private:
    // Robot actual point
    float actual_x;
    float actual_y;
    float actual_theta;
    // Robot goal point
    float goal_x;
    float goal_y;
    // Control cosntants
    float Kv;
    float Kh;
public:
    Point_Controller(float Kv, float Kh);
    // Setters
    void set_actual_point(float actual_x, float actual_y, float actual_theta);
    void set_goal_point(float goal_x, float goal_y);
    // Getters
    float get_velocity();
    float get_angle();
};