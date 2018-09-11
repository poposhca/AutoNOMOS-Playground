#include <iostream>
#include "planner.h"
#include "GridMetadata/AStarAbstraction.h"
#include "search/astar.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner_node");
    auto world = new AStarAbstraction();
    auto explorer = new astar(world);
    planner *p = new planner(world, explorer);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("model/map", 1000, &planner::ReadMap, p);
    //TODO Subscribe to Lalo's nodes
    //TODO Subscribe to LTL node
    ROS_INFO_STREAM("Planner node up & running");
    while(ros::ok)
    {
        ros::spinOnce();
        //call planner
    }
    return 0;
}