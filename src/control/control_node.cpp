#include <ros/ros.h>
#include "controllers/pose_controller.h"
#include "model/ackerman.h"

void UpdateNextPose(const geometry_msgs::Pose2D &msg);
void UpdatePose(const geometry_msgs::Pose2D &msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    //ros::Rate rate(1); 

    //Model
    AckermanModel ackerman(1);

    //Controller
    float kp;
    float ka;
    float kv;
    nhPrivate.param<float>("control_kp", kp, 5);
    //ka > kp
    nhPrivate.param<float>("control_ka", ka, 5.5);
    //Kb < 0
    nhPrivate.param<float>("control_kb", kv, -0.55);
    std::cout << kp << "," << ka << "," << kv << "," << std::endl;
    int control;
    std::cin >> control; 
    Pose_Controller l(nh, kp, ka, kv);

    while(ros::ok())
    {
        ros::spinOnce();
        l.toPolar();
        l.setActualParameters();
        l.SendMessage();
        l.PrintAllParameters();
        //rate.sleep();
    }
}