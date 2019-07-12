#include <istream>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include "./controller/point_controller.h"

#define RADIUS_LIMIT 0.1

Point_Controller *controller;
int car_velocity = -50;
bool is_goal_set = false;
bool is_pose_set = false;
bool is_plann_set = false;
std::vector<int> plann;
// Refactor please
float car_x;
float car_y;

bool check_is_near_point()
{
    float goalX = controller->get_goal_x();
    float goalY = controller->get_goal_y();
    double radious = sqrt(pow(goalX - car_x, 2) + pow(goalY - car_y, 2));
    if(radious <= RADIUS_LIMIT)
        return true;
    else
        return false;
}

void set_next_point(int next_step)
{
    float next_y = car_y + 0.25;
    float next_x;
    if(next_step == 0)
        next_x = car_x;
    if(next_step == 1)
        next_x = car_x + 0.25;
    if(next_step == -1)
        next_x = car_x - 0.25;
    controller->set_goal_point(next_x, next_y);
}

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

void set_next_goal(const std_msgs::Int16 &msg)
{
    is_goal_set = true;
    set_next_point(msg.data);
}

void push_plann(const std_msgs::Int32MultiArray &plann_msg)
{
    ROS_INFO_STREAM("Plann recieved");
    for(int i = 0; i < plann_msg.data.size(); i++)
    {
        plann.push_back( plann_msg.data[i]);
    }
    is_plann_set = true;
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
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Subscriber goal_pose = nh.subscribe("/control/goal", 1000, &set_next_goal);
    ros::Subscriber plann_subscriber = nh.subscribe("/control/plann", 1000, &push_plann);
    ros::Subscriber goal_velocity = nh.subscribe("/control/speed", 1000, &set_car_speed_manual);
    ros::Publisher car_reached_next_goal = nh.advertise<std_msgs::Bool>("/planner/moveNext", 1000);
    ros::Publisher autonomos_v = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", 1000);
    ros::Publisher autonomos_s = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", 1000);
    controller = new Point_Controller(300, 1);
    is_goal_set = false;
    while(ros::ok)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("Control:");
        bool isNearPoint = check_is_near_point();
        if(isNearPoint)
        {
            std_msgs::Bool goal_msg;
            goal_msg.data = true;
            car_reached_next_goal.publish(goal_msg);
        }
        else if(is_goal_set)
        {
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