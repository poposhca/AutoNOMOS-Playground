#include <istream>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include "./controller/point_controller.h"

Point_Controller *controller;

int car_velocity;
bool is_goal_set;
bool is_pose_set;
std::vector<int> plann;
// Refactor please
float car_x;
float car_y;

void autonomos_pose_listener(geometry_msgs::Pose2D msg)
{
    double actual_angle_rad = msg.theta;
    // Transfor angle to degrees, the initial angle is 90
    double actual_gle_degrees = angles::to_degrees(actual_angle_rad);
    controller->set_actual_point(msg.x, msg.y, actual_gle_degrees);
    car_x = msg.x;
    car_y = msg.y;
    is_pose_set = true;
}

void set_next_goal(const geometry_msgs::Pose2D &msg)
{
    is_goal_set = true;
    controller->set_goal_point(msg.x, msg.y);
}

void push_plann(const std_msgs::Int32MultiArray &plann_msg)
{
    std::vector<int> plann = plann_msg.data;
    for(auto state = plann.begin(); state != plann.end(); state++)
        plann.push_back(*state);
}

void set_next_point()
{
    if(plann.size() == 0)
     return;
    auto const next_step = plann.at(0);
    float next_y = car_y + 0.25;
    float next_x;
    if(next_step == 0)
        next_x = car_x;
    if(next_step == 1)
        next_x = car_x + 0.25;
    if(next_step == -1)
        next_x = car_x - 0.25;
    std::cout << next_x << std::endl;
    std::cout << next_y << std::endl;
    controller->set_goal_point(next_x, next_y);
}

void set_car_speed_manual(const std_msgs::Int16 &velocity)
{
    car_velocity = velocity.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_low_node");
    ROS_INFO_STREAM("Low level controller initialized");
    ros::NodeHandle nh;
    // Sub /AutoNOMOS_simulation/real_pose -> geometry_msgs/Pose2D
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Subscriber goal_pose = nh.subscribe("/control/goal", 1000, &set_next_goal);
    ros::Subscriber plann_subscriber = nh.subscribe("/control/plann", 1000, &push_plann);
    ros::Subscriber goal_velocity = nh.subscribe("/control/speed", 1000, &set_car_speed_manual);
    ros::Publisher autonomos_v = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", 1000);
    ros::Publisher autonomos_s = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", 1000);
    controller = new Point_Controller(300, 1);
    is_goal_set = false;
    while(ros::ok)
    {
        ros::spinOnce();
        if(is_pose_set)
        {
            set_next_point();
            ROS_INFO_STREAM("Control signals");
            // Get and publish velocity
            // float velocity = controller->get_velocity() * -1;
            std_msgs::Int16 velocity_msg;
            velocity_msg.data = static_cast<int>(car_velocity);
            autonomos_v.publish(velocity_msg);

            // Get and publish steering
            float angle = controller->get_angle();
            std::cout << "  theta: " << angle << std::endl;
            std_msgs::Int16 steering_msg;
            steering_msg.data = static_cast<int>(angle);
            autonomos_s.publish(steering_msg);
            is_goal_set = false;
        }
    }
    return 0;
}