#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include <string>
#include <vector>
#include <tuple>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include "search_tree.h"
#include "Model/ackerman.h"
#include "Controller/pose_controller.h"
#define PI 3.14159265358979323846

class Explorer
{
private:
    float time;
    float goalx;
    float goaly;
    float goalTheta;
    float car_throttle;
    AckermanModel *model;
    Pose_Controller *controller;
    searchTree *statesTree;
    float goal_accept_zone;
    ros::Publisher constrolSignalPublisher;
    ros::Publisher throttlePublisher;
    bool IsInGoal(float actual_x, float actual_y);

public:
    Explorer(float car_throttle, std::string speedTopic, std::string angleTopic,ros::NodeHandle nh);
    void PublishNextCOntrol(const std::vector<std::tuple<std::string, int>> *plann);
    void StartMoving();
    void SetGoal(float x, float y, float theta);
    void Explor();
};

#endif