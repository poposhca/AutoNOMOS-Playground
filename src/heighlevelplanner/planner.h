#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <tuple>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "GridMetadata/WorldAbstraction.h"
#include "selectGoal/selectGoal.h"
#include "search/search.h"
#include "explore/explorer.h"

#define NUM_STATES 7
#define STATE_WIDTH 20

class planner
{
private:
    WorldAbstraction *world;
    ruteExplorer *searcher;
    Explorer *explorer;
    ros::Publisher pathPublisher;
    void PublicPath(const std::vector<int> *map, const std::vector<int> *path);

public:
    planner(WorldAbstraction *world, ruteExplorer *searcher, Explorer *explorer);
    void ReadLaneState(const std_msgs::Float32MultiArray &laneState);
    void ReadMap(const nav_msgs::OccupancyGrid &map);
    void CreatePlan();
    void test(const std::vector<int> *path, const std::vector<std::tuple<std::string, int>> *plann);
};

#endif