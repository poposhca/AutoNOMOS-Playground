#include <istream>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

double actual_x;
double actual_y;
void autonomos_pose_listener(const geometry_msgs::Pose2D &msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_control_low_node");
    ROS_INFO_STREAM("Tests initialized");
    ros::NodeHandle nh;
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Publisher control_publisher = nh.advertise<geometry_msgs::Pose2D>("/control/goal", 1000);
    ros::Publisher speed_publisher = nh.advertise<std_msgs::Int16>("/control/speed", 1000);
    while(ros::ok)
    {
        ros::spinOnce();

        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = 3;
        pose_msg.y = 2;
        control_publisher.publish(pose_msg);

        std_msgs::Int16 speed_msg;
        speed_msg.data = -100;
        speed_publisher.publish(speed_msg);
    }
}


void autonomos_pose_listener(const geometry_msgs::Pose2D &msg)
{
    actual_x = msg.x;
    actual_y = msg.y;
}