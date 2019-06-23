#include "explorer.h"

Explorer::Explorer(float car_throttle, std::string speedTopic, std::string angleTopic, ros::NodeHandle nh)
{
    this->car_throttle = car_throttle;
    this->constrolSignalPublisher = nh.advertise<std_msgs::Float64>("/local_planner", 1000);
    //this->throttlePublisher = nh.advertise<std_msgs::Int16>(speedTopic, 1000);
    //this->steeringPublisher = nh.advertise<std_msgs::Int16>(angleTopic, 1000);
}

// void Explorer::StartMoving()
// {
//     std_msgs::Int16 throttleMessage;
//     throttleMessage.data = this->car_throttle;
//     this->throttlePublisher.publish(throttleMessage);
// }

void Explorer::PushPlann(const std::vector<std::tuple<std::string, int>> *plann)
{
    if(plann != NULL)
    {
        for(auto state = plann->begin; state != plann->end(); state++)
            this->plann.push_back(*state);
    }
}

// void Explorer::SetGoal(float x, float y, float theta)
// {
//     this->goalx =x;
//     this->goaly = y;
//     this->goalTheta = theta;
// }

// void Explorer::Explor()
// {
//     int actual_state = 0;
//     bool goal_reached = false;
//     while(!goal_reached)
//     {
//         this->controller->UpdateActualPose(this->model->actual_x, this->model->actual_y, this->model->actual_theta);
//         this->controller->UpdateNextPose(this->goalx, this->goaly, this->goalTheta);
//         this->controller->setActualParameters();
//         float v = this->controller->getVelocity();
//         float gamma = this->controller->getSteering();
//         float gamma_radians = gamma * PI / 180;
//         if(this->model->HasValidContrins(v, gamma_radians))
//         {
//             this->model->UpdateParaemters(v, gamma);
//             float* points = this->model->getPoints(this->time);
//             std::cout << "Desplazamiento: " << points[0] << ',' << points[1] << ',' << points[2] << std::endl;
//             goal_reached = IsInGoal(points[0], points[1]);
//         }
//         else
//         {
//             this->goalx -= 5;
//             std::cout << "No se alcanso la ruta, goal actual " << this->goalx << " v " << v << std::endl;
//             int control;
//             std::cin >> control;
//         }
//     }
//     std::cout << "GLORIA" << std::endl;
// }

// bool Explorer::IsInGoal(float actual_x, float actual_y)
// {
//     float dif_x = actual_x * this->goal_accept_zone;
//     float dif_y = actual_y * this->goal_accept_zone;
//     bool acceptable_x = actual_x >= this->goalx - dif_x && actual_x <= this->goalx - dif_x;
//     bool acceptable_y = actual_y >= this->goaly - dif_y && actual_y <= this->goaly - dif_y;
//     return acceptable_x && acceptable_y;
// }