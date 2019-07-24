#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

using namespace std;

double actual_x;
double actual_y;
int actual_plan;
std::string heigh_level_plan;
std::string control_plan;

bool plan_is_set = false;
bool heigh_level_is_set = false;
bool control_plan_is_set = false;

void autonomos_pose_listener(const geometry_msgs::Pose2D &msg)
{
    actual_x = msg.x;
    actual_y = msg.y;
}

void control_listener(const std_msgs::Int16 &plann_msg)
{
    actual_plan = plann_msg.data;
    plan_is_set = true;
}

void heigh_level_listener(const std_msgs::String &heigh_level)
{
    heigh_level_plan = heigh_level.data;
    heigh_level_is_set = true;
}

void control_plan_listener(const std_msgs::String &control)
{
    control_plan = control.data;
    control_plan_is_set = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "experiments_node");
    ros::NodeHandle nh;
    ros::Subscriber autonomos_pose = nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &autonomos_pose_listener);
    ros::Subscriber controller = nh.subscribe("/control/goal", 1000, &control_listener);
    ros::Subscriber heigh_level_plan_subs = nh.subscribe("/model/heighLevelPlan", 1000, &heigh_level_listener);
    ros::Subscriber control_plan_subs = nh.subscribe("/model/controlLevelPlan", 1000, &control_plan_listener);
    // ofstream file;
    // file.open("/home/poposhca/Desktop/results.txt");
    ros::Rate r(5);
    while(ros::ok)
    {
        if(plan_is_set)
        {
            ofstream realPlan;
            realPlan.open("/home/poposhca/Desktop/realPlan.txt", std::ios_base::app);
            realPlan << actual_plan << ",";
            plan_is_set = false;
            realPlan.close();
        }

        if(heigh_level_is_set)
        {
            ofstream statesPlan;
            statesPlan.open("/home/poposhca/Desktop/statesPlan.txt", std::ios_base::app);
            statesPlan << heigh_level_plan << "\n";
            statesPlan.close();
            heigh_level_is_set = false;
        }

        if(control_plan_is_set)
        {
            ofstream controlPLan;
            controlPLan.open("/home/poposhca/Desktop/controlPlan.txt", std::ios_base::app);
            controlPLan << control_plan << "\n";
            controlPLan.close();
            control_plan_is_set = false;
        }

        ros::spinOnce();
    }
    // file.close();
    // realPlan.close();
}