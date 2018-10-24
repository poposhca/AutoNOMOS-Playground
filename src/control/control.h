#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <ros/ros.h>
#include "model/ackerman.h"
#include "controllers/pose_controller.h"

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

public:
    Control(int modelType);
    void NextIteration();
    void UpdateNextPose(const geometry_msgs::Pose2D &msg);
};

#endif