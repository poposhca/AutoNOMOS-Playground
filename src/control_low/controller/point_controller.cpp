#include "./point_controller.h"
#include <iostream>

Point_Controller::Point_Controller(float Kv, float Kh)
{
    this->Kv = Kv;
    this->Kh = Kh;
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
    std::cout << "Delta X: " << delta_x << std::endl;
    std::cout << "Delta Y: " << delta_y << std::endl;
    std::cout << "Theta: " << theta << std::endl;
    std::cout << "Steering: " << steer << std::endl;
    return steer;
}