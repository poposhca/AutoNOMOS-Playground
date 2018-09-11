#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

void FirstCuadrantTest(const ros::Publisher nh)
{
    sensor_msgs::LaserScan dummyScan;
    dummyScan.ranges.resize(360);
    for(int i = 0; i < 90; i++)
        dummyScan.ranges[i] = 3;
    nh.publish(dummyScan);
}

void FrontTest(const ros::Publisher nh)
{
    sensor_msgs::LaserScan dummyScan;
    dummyScan.ranges.resize(360);
    for(int i = 0; i < 90; i++)
        dummyScan.ranges[i] = 3;
    for(int i = 270; i < 360; i++)
        dummyScan.ranges[i] = 3;
    nh.publish(dummyScan);
}

void BackTest(const ros::Publisher nh)
{
    sensor_msgs::LaserScan dummyScan;
    dummyScan.ranges.resize(360);
    for(int i = 90; i < 270; i++)
        dummyScan.ranges[i] = 3;
    nh.publish(dummyScan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MapTesterNode");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Map-tests node publishing tests");
    auto mapPublisher = nh.advertise<sensor_msgs::LaserScan>("/scan", 1000);
    while(ros::ok)
    {
        FirstCuadrantTest(mapPublisher);
    }
    return 0;
}