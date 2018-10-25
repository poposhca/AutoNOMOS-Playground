#include <ros/ros.h>
#include "control.h"

void UpdateNextPose(const geometry_msgs::Pose2D &msg);
void UpdatePose(const geometry_msgs::Pose2D &msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    //ros::Rate rate(1); ros
    Control main_algorithm(1);

    while(ros::ok())
    {
        ros::spinOnce();
        if(main_algorithm.goalSet)
            main_algorithm.NextIteration();
        //rate.sleep();
    }
}