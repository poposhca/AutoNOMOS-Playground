#include <ros/ros.h>
#include <math.h>

class AckermanModel
{

private:

    float wheelBase;
    float v_x;
    float v_y;
    float v_theta;

public:

    AckermanModel(float wheelBase);
    void UpdateParaemters(float velocity, float steering);
    float* getPoints(float delta_time);

};