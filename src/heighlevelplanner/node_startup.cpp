#include <iostream>
#include "planner.h"
#include "GridMetadata/TwoLaneAbstraction.h"
#include "search/astar.h"

#define RATE_HZ 5

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner_node");
    ros::Rate loop_rate(RATE_HZ);
    auto world = new TwoLaneAbstraction();
    auto explorer = new astar(world);
    planner *p = new planner(world, explorer);
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe("/localization_array", 1000, &planner::ReadLaneState, p);
    ros::Subscriber sub2 = nh.subscribe("model/map", 1000, &planner::ReadMap, p);
    //TODO Subscribe to LTL node
    ROS_INFO_STREAM("Planner node up & running");
    while(ros::ok)
    {
        ros::spinOnce();
        p->CreatePlan();
        loop_rate.sleep();
    }
    return 0;
}