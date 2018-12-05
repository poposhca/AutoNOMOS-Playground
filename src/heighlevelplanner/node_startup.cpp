#include <iostream>
#include "planner.h"
#include "GridMetadata/TwoLaneAbstraction.h"
#include "search/astar.h"
#include "explore/explorer.h"

#define RATE_HZ 5

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner_node");
    ros::NodeHandle nh;
    auto world = new TwoLaneAbstraction();
    auto searcher = new astar(world);
    auto explorer = new Explorer(-8, nh);
    planner *p = new planner(world, searcher, explorer);
    ros::Subscriber sub1 = nh.subscribe("/localization_array", 1000, &planner::ReadLaneState, p);
    ros::Subscriber sub2 = nh.subscribe("model/map", 1000, &planner::ReadMap, p);
    //TODO Subscribe to LTL node
    ROS_INFO_STREAM("Planner node up & running");
    ros::Rate loop_rate(RATE_HZ);
    while(ros::ok)
    {
        ros::spinOnce();
        p->CreatePlan();
        loop_rate.sleep();
    }
    return 0;
}