#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define RATE_HZ 1

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "State_mock_node");
    cout << "State node up" << endl;
    ros::NodeHandle nh;
    ros::Publisher state_publisher = nh.advertise<std_msgs::Float32MultiArray>("/localization_array_test", 1000);
    while(ros::ok)
    {
        vector<float> data_vector;
        data_vector.push_back(4);

        std_msgs::Float32MultiArray msg_vector;
        msg_vector.data = data_vector;

        state_publisher.publish(msg_vector);
    }
}