#include <iostream>
#include "planner.h"
#include "GridMetadata/TwoLaneAbstraction.h"
#include "search/astar.h"
#include "explore/explorer.h"

#define RATE_HZ 5

using namespace std;

int main(int argc, char **argv)
{
    //Init node
    ros::init(argc, argv, "Planner_node");
    ros::NodeHandle priv_nh("~");
    ros::NodeHandle nh;

    //Set parameters
    std::string speedTopic;
    std::string steerTopic;
    priv_nh.param<std::string>("planner/car_speed", speedTopic, "/AutoNOMOS_mini/manual_control/speed");
    priv_nh.param<std::string>("planner/car_speed", steerTopic, "/AutoNOMOS_mini/manual_control/steering");

    //Set planner components
    auto world = new TwoLaneAbstraction();
    auto searcher = new astar(world);
    auto explorer = new Explorer(-100, speedTopic, steerTopic, nh);
    auto *planner_algorithm = new planner(world, searcher, explorer);
    //TODO Subscribe to LTL node

    //Subscribe to topics
    ros::Subscriber subscribe_localization = nh.subscribe("/localization_array_test", 1000, &planner::ReadLaneState, planner_algorithm);
    ros::Subscriber subscribe_map = nh.subscribe("model/map", 1000, &planner::ReadMap, planner_algorithm);

    //Start main algorithmr
    ROS_INFO_STREAM("Planner node up & running");
    ros::Rate loop_rate(RATE_HZ);
    while(ros::ok)
    {
        ros::spinOnce();
        planner_algorithm->CreatePlan();
        loop_rate.sleep();
    }
    return 0;
}