#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "GridMetadata/WorldAbstraction.h"
#include "search/search.h"

#define NUM_STATES 7
#define STATE_WIDTH 20

class planner
{
private:
    WorldAbstraction *world;
    ruteExplorer *explorer;
    ros::Publisher pathPublisher;
    void PublicPath(const std::vector<int> *map, const std::vector<int> *path);

public:
    planner(WorldAbstraction *world, ruteExplorer *explorer);
    void ReadLaneState(const std_msgs::Float32MultiArray &laneState);
    void ReadMap(const nav_msgs::OccupancyGrid &map);
    void CreatePlan();
    void test(const std::vector<int> *path);
};

#endif