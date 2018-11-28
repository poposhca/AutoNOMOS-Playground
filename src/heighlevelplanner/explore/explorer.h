#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include <string>
#include <vector>
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
    AckermanModel *model;
    Pose_Controller *controller;
    searchTree *statesTree;
    float goal_accept_zone;
    bool IsInGoal(float actual_x, float actual_y);

public:
    Explorer(float car_throttle, float map_resolution, std::vector<std::string> *plan);
    void SetGoal(float x, float y, float theta);
    void Explor();
};

#endif