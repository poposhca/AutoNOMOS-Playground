#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_low_node");
    ROS_INFO_STREAM("Low level controller initialized");
}