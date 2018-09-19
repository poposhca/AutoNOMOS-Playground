#include "planner.h"
#include "GridMetadata/AStarAbstraction.h"

planner::planner(WorldAbstraction *world, ruteExplorer *explorer)
{
    ros::NodeHandle nh;
    this->estado_actual = 0;
    this->world = world;
    this->explorer = explorer;
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
}

void planner::ReadLaneState(const std_msgs::Float32MultiArray &laneState)
{
    ROS_INFO_STREAM("Testing reading lanes states");

    int estadoPrevio = this->estado_actual;
    float max=0;
    for(int i = 0; i < NUM_STATES * STATE_WIDTH; i++) {
        if(laneState.data[i]>max) {
            max=laneState.data[i];
        }
    }

    int countStates=0;
    int state = -1;
    for(int i = NUM_STATES * STATE_WIDTH - 1; i >= 0; i--) {
        if(laneState.data[i]==max) {
            int temp_state = (int)floor(i / STATE_WIDTH);
            if (temp_state != state){
                state = temp_state;
                countStates++;
            }
        }
    }

    if (countStates==1)
        this->estado_actual = state;
    else
        this->estado_actual = -1; // no se pudo determinar el estado, ya que hay mas de uno posible

    ROS_INFO_STREAM("Estado actual: " + this->estado_actual);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    std::cout << "Reading map" << std::endl;
    this->world->setMap(map);
}

void planner::PublicPath(const nav_msgs::OccupancyGrid &map, const std::vector<int> *path)
{
    nav_msgs::OccupancyGrid pathMap;
    pathMap.data.resize(map.info.width * map.info.height);
    fill(pathMap.data.begin(), pathMap.data.end(), -1);
    for(auto i = path->begin(); i != path->end(); i++)
        pathMap.data[*i] = 200;
    this->pathPublisher.publish(pathMap);
}

void planner::test()
{
    if(this->world->getIsMapSet())
    {
        ROS_INFO_STREAM("Performing testings:");
        //dynamic_cast<AStarAbstraction*>(this->world)->Test();
        auto path = explorer->getRute(0, 100);
        for(auto i = path->begin(); i != path->end(); i++)
            std::cout << *i << ", ";
        std::cout << "" << std::endl;
        std::cout << "================================" << std::endl;
    }
    else ROS_INFO_STREAM("Not testing");
}