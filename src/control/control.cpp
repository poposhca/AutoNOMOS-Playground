#include "control.h"

Control::Control(int modelType)
{
    this->time = 5;
    ros::NodeHandle nh;
    this->model = new AckermanModel(1);
    this->controller = new Pose_Controller();
    this->GoalPose = nh.subscribe("robot/next_pose", 1000, &Control::UpdateNextPose, this);
}

void Control::NextIteration()
{
    this->controller->UpdateActualPose(0,0,0);
    this->controller->UpdateNextPose(this->goalx, this->goaly, this->goalTheta);
    //TODO: This part must be inside controller
    this->controller->toPolar();
    this->controller->setActualParameters();
    //
    float v = this->controller->getVelocity();
    float gamma = this->controller->getSteering();
    this->model->UpdateParaemters(v, gamma);
    float* points = this->model->getPoints(this->time);
    this->controller->PrintAllParameters();
    std::cout << points[0] << ',' << points[1] << ',' << points[2] << std::endl;
}

void Control::UpdateNextPose(const geometry_msgs::Pose2D &msg)
{
    this->goalx = msg.x;
    this->goaly = msg.y;
    this->goalTheta = msg.theta;
}