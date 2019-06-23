#include "./point_controller.h"
#include <iostream>

Point_Controller::Point_Controller(float Kv, float Kh)
{
    this->Kv = Kv;
    this->Kh = Kh;
    this->is_goal_setted = false;
}

void Point_Controller::set_actual_point(float actual_x, float actual_y, float actual_theta)
{
    this->actual_x = actual_x;
    this->actual_y = actual_y;
    this->actual_theta = actual_theta;
}

void Point_Controller::set_goal_point(float goal_x, float goal_y)
{
    this->goal_x = goal_x;
    this->goal_y = goal_y;
    this->is_goal_setted = true;
}

float Point_Controller::get_velocity()
{
    float delta_x = powf(this->goal_x - this->actual_x, 2.0f);
    float delta_y = powf(this->goal_y - this->actual_y, 2.0f);
    return this->Kv * sqrtf(delta_x + delta_y);
}

float Point_Controller::get_angle()
{
    float delta_x = this->goal_x - this->actual_x;
    if(delta_x == 0)
        return 0;
    float delta_y = this->goal_y - this->actual_y;
    float theta = angles::to_degrees(atanf(delta_y / delta_x));
    float steer;
    if(theta < 0)
        steer = this->Kh * (90 + theta);
    steer = this->Kh * (90 - theta);
    return steer;
}

float Point_Controller::get_goal_x()
{
    return this->goal_x;
}

float Point_Controller::get_goal_y()
{
    return this->goal_y;
}

bool Point_Controller::get_is_goal_setted()
{
    return this->is_goal_setted;
}