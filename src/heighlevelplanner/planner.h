#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "GridMetadata/WorldAbstraction.h"
#include "search/search.h"

class planner
{
private:
    WorldAbstraction *world;
    ruteExplorer *explorer;
    ros::Publisher pathPublisher;

public:
    planner(WorldAbstraction *world, ruteExplorer *explorer);
    void ReadMap(const nav_msgs::OccupancyGrid &map);
    void PublicPath(const nav_msgs::OccupancyGrid &map, const std::vector<int> *path);
    void testing();
};

#endif