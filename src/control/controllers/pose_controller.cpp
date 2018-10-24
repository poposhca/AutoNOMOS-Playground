#include "pose_controller.h"
    
Pose_Controller::Pose_Controller()
{
    ros::NodeHandle nhPrivate("~");

    //TODO: THIS GOES TO THE AUTONOMOS MODEL
    //this->pubVel = this->nh.advertise<std_msgs::Int16>("AutoNOMOS_mini/manual_control/speed", 1000);
    //this->pubSteer = this->nh.advertise<std_msgs::Int16>("AutoNOMOS_mini/manual_control/steering", 1000);
    //this->SimulationPose = this->nh.subscribe("/AutoNOMOS_simulation/real_pose", 1000, &Pose_Controller::UpdatePose, this);

    nhPrivate.param<float>("control_kp", this->kp, 5);
    //ka > kp
    nhPrivate.param<float>("control_ka", this->ka, 5.5);
    //Kb < 0
    nhPrivate.param<float>("control_kb", this->kb, -0.55);
    this->v = 0;
    this->gamma = 0;
}

void Pose_Controller::toPolar()
{
    //Calcular ro
    float dx = pow(x - goalx, 2);
    float dy = pow(y - goaly, 2);
    ro = sqrt(dx + dy);

    //Calcular alpha
    alpha = atan(dy / dx) - theta;

    //Calcular beta
    beta =  - theta - alpha + angles::from_degrees(goalTheta);
}

void Pose_Controller::setActualParameters()
{
    v = kp * ro;
    gamma = ka * alpha + kb * beta;
}

void Pose_Controller::UpdateNextPose(float x, float y, float theta)
{
    this->goalx = x;
    this->goaly = y;
    this->goalTheta = theta;
}

void Pose_Controller::UpdateActualPose(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// void Pose_Controller::SendMessage()
// {
//     std_msgs::Int16 msgv;
//     msgv.data = -1 * v;
//     std_msgs::Int16 msgs;
//     msgs.data = angles::to_degrees(gamma);
//     pubVel.publish(msgv);
//     pubSteer.publish(msgs);
// }

float Pose_Controller::getVelocity()
{
    return this->v;
}

float Pose_Controller::getSteering()
{
    return this->gamma;
}

void Pose_Controller::PrintAllParameters()
{
    std::cout << "CONTROL VARIABLES:" << std::endl;
    std::cout << "kp: " << this->kp << " ka: " << this->ka << " kb: " << this->kb << std::endl;
    std::cout << "GOAL POSE:" << std::endl;
    std::cout << "x: " << this->goalx << " y: " << this->goaly << " theta: " << this->goalTheta << std::endl;
    std::cout << "ACTUAL POSE:" << std::endl;
    std::cout << "x: " << this->x << " y: " << this->y << " theta: " << this->theta << std::endl;
    std::cout << "CONTROL SIGNALS:" << std::endl;
    std::cout << "v: " << this->v << " gamma: " << this->gamma << std::endl;
}