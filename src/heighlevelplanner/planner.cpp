#include "planner.h"

planner::planner(WorldAbstraction *world, ruteExplorer *searcher, Explorer *explorer)
{
    ros::NodeHandle nh;
    this->world = world;
    this->searcher = searcher;
    this->explorer = explorer;
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
}

void planner::ReadLaneState(const std_msgs::Float32MultiArray &laneState)
{
    this->world->setState(laneState);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    //std::cout << "Reading map" << std::endl;
    this->world->setMap(map);
    this->world->Compute_Abstraction();
}

void planner::PublicPath(const std::vector<int> *map, const std::vector<int> *path)
{
    std::cout << "Publishing map de: " << map->size() << std::endl;
    nav_msgs::OccupancyGrid pathMap;
    pathMap.data.resize(map->size());
    fill(pathMap.data.begin(), pathMap.data.end(), 0);
    for(auto i = path->begin(); i != path->end(); i++)
        pathMap.data[*i] = 200;
    pathMap.info.origin.position.x = 0;
    pathMap.info.origin.position.y = 0;
    pathMap.info.origin.position.z = 0;
    pathMap.info.origin.orientation.x = 0;
    pathMap.info.origin.orientation.y = 0;
    pathMap.info.origin.orientation.z = 0;
    pathMap.info.origin.orientation.w = 0;
    pathMap.info.resolution = this->world->getResolution();
    pathMap.info.width = this->world->getWidth();
    pathMap.info.height = this->world->getHeight();
    this->pathPublisher.publish(pathMap);
}

void planner::CreatePlan()
{
    explorer->StartMoving();  
     if(this->world->getIsMapSet())
     {
        int start = ((this->world->getHeight() / 2) - 1) * this->world->getWidth() + (this->world->getWidth() / 2);
        //TODO: add state verification cycle
        SelectGoal goalSelector(this->world);
        int goal = goalSelector.getGoal();
        auto path = this->searcher->getRute(start, goal);
        auto plann = this->world->getStatesChain(path);

        //Public control signal
        this->explorer->PublishNextCOntrol(plann);

        //Public to Rviz
        this->PublicPath(this->world->getMap(), path);

        //Logg results for testing
        //this->test(path, plann);
     }
}

void planner::test(const std::vector<int> *path, const std::vector<std::tuple<std::string, int>> *plann)
{
    ROS_INFO_STREAM("Logging results");

    if(path == NULL)
        return;
    std::cout << "Searched states:" << std::endl;
    for(auto i = path->begin(); i != path->end(); i++)
        std::cout << *i << ", ";
    std::cout << "" << std::endl;

    if(plann == NULL)
        return;
    std::cout << "Hiegh level plan:" << std::endl;
    for(auto i = plann->begin(); i != plann->end(); i++)
         std::cout << "(" << std::get<0>(*i) << "," << std::get<1>(*i) << "), ";
    std::cout << "" << std::endl;

    std::cout << "================================" << std::endl;
    char control;
    std::cin >> control;
}