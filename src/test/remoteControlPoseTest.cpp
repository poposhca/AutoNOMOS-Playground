#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

void PrintRobotPose(const geometry_msgs::Pose2D &msg)
{
    ROS_INFO_STREAM("Actual AutoNomos pose:");
    cout << "\t" << msg.x << "," << msg.y << "," << msg.theta << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PoseRemoteControllTesterNode");
    ROS_INFO_STREAM("Pose remote control node publishing test");
    ros::NodeHandle nh;
    ros::Publisher posePub = nh.advertise<geometry_msgs::Pose2D>("robot/next_pose", 1000);
    ros::Subscriber poseSub = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &PrintRobotPose);
    geometry_msgs::Pose2D testPose;
    testPose.x = 0.0;
    testPose.y = 10.0;
    testPose.theta = 0.0;
    while(ros::ok())
    {
        posePub.publish(testPose);
    }
    return 0;
}