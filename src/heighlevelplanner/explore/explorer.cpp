// #include "explorer.h"

// Explorer::Explorer(float car_throttle, std::string speedTopic, std::string angleTopic,ros::NodeHandle nh)
// {
//     //this->time = map_resolution / car_throttle;
//     this->car_throttle = car_throttle;
//     this->model = new AckermanModel(1);
//     this->controller = new Pose_Controller();
//     this->statesTree = new searchTree();
//     this->constrolSignalPublisher = nh.advertise<std_msgs::Float64>("/local_planner", 1000);
//     this->throttlePublisher = nh.advertise<std_msgs::Int16>(speedTopic, 1000);
//     this->steeringPublisher = nh.advertise<std_msgs::Int16>(angleTopic, 1000);
// }

// void Explorer::StartMoving()
// {
//     std_msgs::Int16 throttleMessage;
//     throttleMessage.data = this->car_throttle;
//     this->throttlePublisher.publish(throttleMessage);
// }

// void Explorer::PublishNextCOntrol(const std::vector<std::tuple<std::string, int>> *plann)
// {
//     int controlSignal = std::get<1>(plann->at(1));
//     std_msgs::Int16 controlMessage;
//     if(controlSignal == 0)
//         controlMessage.data = 90;
//     else
//         controlMessage.data = controlSignal == -1 ? -30 + 90 : 30 + 90;
//     this->steeringPublisher.publish(controlMessage);
//     // controlMessage.data = controlSignal;
//     // this->constrolSignalPublisher.publish(controlMessage);
// }

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