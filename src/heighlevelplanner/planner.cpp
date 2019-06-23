#include "planner.h"

planner::planner(WorldAbstraction *world, ruteExplorer *searcher)
{
    ros::NodeHandle nh;
    this->world = world;
    this->searcher = searcher;
    this->plannPublisher = nh.advertise<std_msgs::Int32MultiArray>("/control/plann", 1000);
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
    this->statesPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/states", 1000);
    this->automaton = new ltl_Automaton();
    //this->automaton->create_automaton("!F(RC & X(RR))");
    this->automaton->create_automaton("RC");
}

void planner::CreatePlan()
{
    if(this->world->getIsMapSet() && this->world->getStateIsSet())
    {
        this->world->Compute_Abstraction();
        int start = ((this->world->getHeight() / 2) - 1) * this->world->getWidth() + (this->world->getWidth() / 2);
        int goal;
        std::vector<int> *path;
        std::vector<std::tuple<std::string, int>> *plann;
        bool ltl_validation;
        do
        {
            SelectGoal goalSelector(this->world);
            goal = goalSelector.getGoal();
            path = this->searcher->getRute(start, goal);
            plann = this->world->getStatesChain(path);
            ltl_validation = this->automaton->evaluate_formula(plann);
        } while (!ltl_validation);   
        //Push current plan to explorer
        this->PublicPlann(plann);
        //Public to Rviz
        this->PublicPath(this->world->getMap(), path);
        this->PublicStates(this->world->getMap(), this->world->getMapStates());
        //Logg results for testing
        this->test(path, plann);
    }
}

void planner::ReadLaneState(const std_msgs::Float32MultiArray &loacalization_array)
{
    auto state = loacalization_array.data.at(0);
    this->world->setState(state);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    this->world->setMap(map);
}

void planner::PublicPlann(const std::vector<std::tuple<std::string, int>> *plann)
{
    if(plann == NULL)
        return;
    std_msgs::Int32MultiArray plann_msg;
    plann_msg.data.resize(plann->size());
    for(int i = 0; i < plann->size(); i++)
    {
        auto signal = plann->at(i);
        int control_signal = std::get<1>(signal);
        plann_msg.data[i]= control_signal;
    }
    for(int i = 0; i < plann->size(); i++)
        std::cout << plann_msg.data[i] << std::endl;
    this->plannPublisher.publish(plann_msg);
}

void planner::PublicPath(const std::vector<int> *map, const std::vector<int> *path)
{
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

void planner::PublicStates(const std::vector<int> *map, const std::vector<int> *states)
{
    nav_msgs::OccupancyGrid pathMap;
    pathMap.data.resize(map->size());
    fill(pathMap.data.begin(), pathMap.data.end(), 0);
    for(int i = 0; i < states->size(); i++)
    {
        int state = states->at(i);
        int cellValue = state * 100;
        pathMap.data[i] = cellValue;
    }
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
    this->statesPublisher.publish(pathMap);
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
    // char control;
    // std::cin >> control;
}