#include <istream>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

void autonomos_pose_listener(const geometry_msgs::Pose2D &msg)
{
    ROS_INFO_STREAM("Actual AutoNomos pose (angle in RAD):");
    cout << "\t" << "x:" << msg.x << ", y:"  << msg.y << ", theta:" << msg.theta << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_low_node");
    ROS_INFO_STREAM("Low level controller initialized");
    ros::NodeHandle nh;
    // Sub /AutoNOMOS_simulation/real_pose -> geometry_msgs/Pose2D
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Publisher autonomos_v = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", 1000);
    ros::Publisher autonomos_s = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", 1000);
    while(ros::ok)
    {
        ros::spinOnce();
        std_msgs::Int16 velocity_msg;

        // Car go forward with negative value
        // velocity_msg.data = -50;
        // autonomos_v.publish(velocity_msg);
        // std_msgs::Int16 steering_msg;

        // 0 degrees is equal to 90 degrees, car reads degrees
        // steering_msg.data = 90 + 15;
        // autonomos_s.publish(steering_msg);
    }
    return 0;
}