#include <ros/ros.h>
#include <cmath>

class AckermanModel
{

private:
    //Properties
    float wheelBase;
    float v_x;
    float v_y;
    float v_theta;
    //Model constraints
    float max_angle;
    float min_angle;
    float max_vel;
    float min_vel;
    //Methods

public:
   //Actual pose
    float actual_x;
    float actual_y;
    float actual_theta;
    AckermanModel(float wheelBase);
    bool HasValidContrins(float v, float steering);
    void UpdateParaemters(float velocity, float steering);
    float* getPoints(float delta_time);
};