#include "control.h"

Control::Control(int modelType)
{
    this->time = 5;
    ros::NodeHandle nh;
    this->model = new AckermanModel(1);
    this->controller = new Pose_Controller();
    this->GoalPose = nh.subscribe("robot/next_pose", 1000, &Control::UpdateNextPose, this);
    this->goalSet = false;
}

void Control::NextIteration()
{
    bool goal_reached = false;
    while(!goal_reached)
    {
        this->controller->UpdateActualPose(this->model->actual_x, this->model->actual_y, this->model->actual_theta);
        this->controller->UpdateNextPose(this->goalx, this->goaly, this->goalTheta);
        this->controller->setActualParameters();
        float v = this->controller->getVelocity();
        float gamma = this->controller->getSteering();
        float gamma_radians = gamma * PI / 180;
        if(this->model->HasValidContrins(v, gamma_radians))
        {
            this->model->UpdateParaemters(v, gamma);
            float* points = this->model->getPoints(this->time);
            std::cout << "Desplazamiento: " << points[0] << ',' << points[1] << ',' << points[2] << std::endl;
            goal_reached = IsInGoal(points[0], points[1]);
        }
        else
        {
            this->goalx -= 5;
            std::cout << "No se alcanso la ruta, goal actual " << this->goalx << " v " << v << std::endl;
            int control;
            std::cin >> control;
        }
    }
    std::cout << "GLORIA" << std::endl;
}

void Control::UpdateNextPose(const geometry_msgs::Pose2D &msg)
{
    this->goalSet = true;
    this->goalx = msg.x;
    this->goaly = msg.y;
    this->goalTheta = msg.theta;
}

bool Control::IsInGoal(float actual_x, float actual_y)
{
    float dif_x = actual_x * this->goal_accept_zone;
    float dif_y = actual_y * this->goal_accept_zone;
    bool acceptable_x = actual_x >= this->goalx - dif_x && actual_x <= this->goalx - dif_x;
    bool acceptable_y = actual_y >= this->goaly - dif_y && actual_y <= this->goaly - dif_y;
    return acceptable_x && acceptable_y;
}