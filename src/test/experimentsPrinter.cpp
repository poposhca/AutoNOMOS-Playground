#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

double actual_x;
double actual_y;
std::vector<int> plann;
bool plan_is_set = false;

void autonomos_pose_listener(const geometry_msgs::Pose2D &msg)
{
    actual_x = msg.x;
    actual_y = msg.y;
}

void push_plann(const std_msgs::Int32MultiArray &plann_msg)
{
    ROS_INFO_STREAM("Plann recieved");
    plan_is_set = true;
    for(int i = 0; i < plann_msg.data.size(); i++)
    {
        plann.push_back(plann_msg.data[i]);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "experiments_node");
    ros::NodeHandle nh;
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Subscriber plann_subscriber = nh.subscribe("/control/plann", 1000, &push_plann);
    ofstream file;
    file.open("/home/poposhca/Desktop/results.txt");
    while(ros::ok)
    {
        ros::spinOnce();
        //Print plan
        if(plan_is_set)
        {
            for(int i = 0; i < plann.size(); i++)
            {
                file << plann[i] << ",";
            }
            file << "\n";
        }
    }
    file.close();
}