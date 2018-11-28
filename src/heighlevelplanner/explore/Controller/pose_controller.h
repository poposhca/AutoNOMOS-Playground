#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <angles/angles.h>

class Pose_Controller
{

private:

    ros::NodeHandle nh;

public:
    
    //Ros variables
    ros::Publisher pubVel;
    ros::Publisher pubSteer;
    ros::Subscriber SimulationPose;

    //Actual pose
    float x;
    float y;
    float theta;

    //Goal pose
    float goalx;
    float goaly;
    float goalTheta; //In radians

    //Polar coordenates
    float ro;
    float alpha;
    float beta;

    //Actual parameters
    float v;
    float gamma;

    //Control parameters
    float kp;
    float ka;
    float kb;
    
    Pose_Controller();
    void toPolar();
    //void SendMessage();
    float getVelocity();
    float getSteering();
    void setActualParameters();
    void UpdateNextPose(float x, float y, float theta);
    void UpdateActualPose(float x, float y, float theta);
    void PrintAllParameters();

};