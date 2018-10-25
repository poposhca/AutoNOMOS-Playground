#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <ros/ros.h>
#include "model/ackerman.h"
#include "controllers/pose_controller.h"
#define PI 3.14159265358979323846

class Control
{
private:
    float time;
    float goalx;
    float goaly;
    float goalTheta;
    ros::Subscriber GoalPose;
    AckermanModel *model;
    Pose_Controller *controller;
    float goal_accept_zone;
    bool IsInGoal(float actual_x, float actual_y);

public:
    bool goalSet;
    Control(int modelType);
    void NextIteration();
    void UpdateNextPose(const geometry_msgs::Pose2D &msg);
};

#endif