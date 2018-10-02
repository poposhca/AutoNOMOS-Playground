#include "planner.h"
#include "GridMetadata/AStarAbstraction.h"

planner::planner(WorldAbstraction *world, ruteExplorer *explorer)
{
    ros::NodeHandle nh;
    this->world = world;
    this->explorer = explorer;
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
}

void planner::ReadLaneState(const std_msgs::Float32MultiArray &laneState)
{
    this->world->setState(laneState);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    std::cout << "Reading map" << std::endl;
    this->world->setMap(map);
}

void planner::PublicPath(const std::vector<int> *map, const std::vector<int> *path)
{
    std::cout << "Publishing map de: " << map->size() << std::endl;
    nav_msgs::OccupancyGrid pathMap;
    pathMap.data.resize(map->size());
    fill(pathMap.data.begin(), pathMap.data.end(), -1);
    for(auto i = path->begin(); i != path->end(); i++)
        pathMap.data[*i] = 200;
    pathMap.info.origin.position.x = 0;
    pathMap.info.origin.position.y = 0;
    pathMap.info.origin.position.z = 0;
    pathMap.info.origin.orientation.x = 0;
    pathMap.info.origin.orientation.y = 0;
    pathMap.info.origin.orientation.z = 0;
    pathMap.info.origin.orientation.w = 0;
    pathMap.info.resolution = this->world->getResolution();
    pathMap.info.width = this->world->getWidth();
    pathMap.info.height = this->world->getHeight();
    this->pathPublisher.publish(pathMap);
}

void planner::CreatePlan()
{
     if(this->world->getIsMapSet())
     {
        auto path = this->explorer->getRute(36, 2292);
        this->PublicPath(this->world->getMap(), path);
        this->test(path);
     }
}

void planner::test(const std::vector<int> *path)
{
    ROS_INFO_STREAM("Performing testings");
    for(auto i = path->begin(); i != path->end(); i++)
        std::cout << *i << ", ";
    std::cout << "" << std::endl;
    std::cout << "================================" << std::endl;
    char control;
    std::cin >> control;
}