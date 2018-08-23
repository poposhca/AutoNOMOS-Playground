#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "GridMetadata/WorldAbstraction.h"

#include "search/search.h"
#include "search/astar.h"

#define DEBUG

using namespace std;

class planner
{
private:
    WorldAbstraction *World;
    ruteExplorer *plannerSearcher;
    ros::Publisher pathPublisher;

public:

    planner()
    {
        ros::NodeHandle nh;
        this->World = new WorldAbstraction();
        this->plannerSearcher = new astar();
        this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
    }

    //void ReadLTL(const nda &ltl)

    void ReadMap(const nav_msgs::OccupancyGrid &map)
    {
        std::cout << "probando mapa" << "std::endl";
        this->World->setMap(map);
        this->World->Tests();
    }

    void PublicPath(const nav_msgs::OccupancyGrid &map, const std::vector<int> *path)
    {
        nav_msgs::OccupancyGrid pathMap;
        pathMap.data.resize(map.info.width * map.info.height);
        fill(pathMap.data.begin(), pathMap.data.end(), -1);
        for(auto i = path->begin(); i != path->end(); i++)
            pathMap.data[*i] = 200;
        this->pathPublisher.publish(pathMap);
    }

    void testing()
    {
        (dynamic_cast<astar*>(this->plannerSearcher))->Test();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner_node");
    planner *p = new planner;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("model/map", 1000, &planner::ReadMap, p);
    while(ros::ok)
    {
        ros::spinOnce(); // -> Get map, Get LTL, build abstract
        //call planner
    }
    return 0;
}