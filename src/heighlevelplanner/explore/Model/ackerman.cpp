#include "ackerman.h"

AckermanModel::AckermanModel(float wheelBase)
{
    ros::NodeHandle nhPrivate("~");
    this->wheelBase = wheelBase;
    this->v_x = 0;
    this->v_y = 0;
    this->v_theta = 0;
    this->actual_x = 0;
    this->actual_y = 0;
    this->actual_theta = 0;
    nhPrivate.param<float>("model_min_vel", this->min_vel, 0);
    nhPrivate.param<float>("model_max_vel", this->max_vel, 10);
    nhPrivate.param<float>("model_min_angle", this->min_angle, -0.785398);
    nhPrivate.param<float>("model_max_angle", this->min_angle, 0.785398);
}

void AckermanModel::UpdateParaemters(float velocity, float steering_radians)
{
    this->v_x = velocity * cos(steering_radians);
    this->v_y = velocity * sin(steering_radians);
    this->v_theta = (velocity / this->wheelBase) * tan(v_theta);
}

bool AckermanModel::HasValidContrins(float v, float steering)
{
    bool hasProperVel = v >= this->min_vel && v <= this->max_angle;
    bool hasProperAngle = steering >= this->min_angle && v <= this->max_angle;
    return hasProperAngle && hasProperVel;
}

float* AckermanModel::getPoints(float delta_time)
{
    //float *points = new float[3];
    this->actual_x = v_x * delta_time + this->actual_x;
    this->actual_y = v_y * delta_time + this->actual_y;
    this->actual_theta = v_theta * delta_time + this->actual_theta;
    return new float[3] {this->actual_x, this->actual_y, this->actual_theta};
}