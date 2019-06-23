#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include <string>
#include <vector>
#include <tuple>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
// #include "Model/ackerman.h"
// #define PI 3.14159265358979323846

class Explorer
{
private:
    std::vector<std::tuple<std::string, int>> plann;
    float car_throttle;
    ros::Publisher constrolSignalPublisher;
//     float time;
//     float goalx;
//     float goaly;
//     float goalTheta;
//     AckermanModel *model;
//     searchTree *statesTree;
//     float goal_accept_zone;
//     ros::Publisher throttlePublisher;
//     ros::Publisher steeringPublisher;
//     bool IsInGoal(float actual_x, float actual_y);

public:
    Explorer(float car_throttle, std::string speedTopic, std::string angleTopic, ros::NodeHandle nh);
    void PushPlann(const std::vector<std::tuple<std::string, int>> *plann);
//     void StartMoving();
//     void SetGoal(float x, float y, float theta);
//     void Explor();
};

#endif