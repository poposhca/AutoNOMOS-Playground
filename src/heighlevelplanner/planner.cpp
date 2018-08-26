#include "planner.h"

planner::planner(WorldAbstraction *world, ruteExplorer *explorer)
{
    ros::NodeHandle nh;
    this->world = world;
    this->explorer = explorer;
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    this->world->setMap(map);
}

void planner::PublicPath(const nav_msgs::OccupancyGrid &map, const std::vector<int> *path)
{
    nav_msgs::OccupancyGrid pathMap;
    pathMap.data.resize(map.info.width * map.info.height);
    fill(pathMap.data.begin(), pathMap.data.end(), -1);
    for(auto i = path->begin(); i != path->end(); i++)
        pathMap.data[*i] = 200;
    this->pathPublisher.publish(pathMap);
}