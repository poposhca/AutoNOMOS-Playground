#include <istream>
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <vector>
#include <tuple>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

// Next Goal variables
// Carril izquierdo: 0.31, derecho: 0.75
vector<tuple<double, double>> plann;
int actual_step;
bool is_near_point;
void autonomos_pose_listener(const geometry_msgs::Pose2D &msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_control_low_node");
    ROS_INFO_STREAM("Tests initialized");
    ros::NodeHandle nh;
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Publisher control_publisher = nh.advertise<geometry_msgs::Pose2D>("/control/goal", 1000);
    ros::Publisher speed_publisher = nh.advertise<std_msgs::Int16>("/control/speed", 1000);

    //Make fake plann
    plann.push_back(make_tuple(0.75, 1.0));
    plann.push_back(make_tuple(0.31, 2.0));
    plann.push_back(make_tuple(0.75, 3.0));
    plann.push_back(make_tuple(0.31, 4.0));
    actual_step = 0;
    is_near_point = false;

    while(ros::ok)
    {
        ros::spinOnce();

        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = get<0>(plann.at(actual_step));
        pose_msg.y = get<1>(plann.at(actual_step));
        control_publisher.publish(pose_msg);

        std_msgs::Int16 speed_msg;
        if(is_near_point)
            speed_msg.data = 0;
        else
            speed_msg.data = -50;
        speed_publisher.publish(speed_msg);
    }
}


void autonomos_pose_listener(const geometry_msgs::Pose2D &msg)
{
    double actual_x = msg.x;
    double actual_y = msg.y;
    double goalX = get<0>(plann.at(actual_step));
    double goalY = get<0>(plann.at(actual_step));
    double radious = sqrt(pow(goalX - actual_x, 2) + pow(goalY - actual_y, 2));
    cout << "X: " << actual_x << " Y: " << actual_y << radious << endl;
    cout << "Radio " << radious << endl;
    bool car_at_end = actual_step == plann.size() - 1;
    if(radious <= 0.3 && car_at_end)
        is_near_point = true;
    else if(radious <= 0.3)
        actual_step++;
}