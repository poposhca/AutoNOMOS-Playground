#include "planner.h"

planner::planner(WorldAbstraction *world, ruteExplorer *searcher)
{
    ros::NodeHandle nh;
    this->searchedPlan = new SearchedPlan;
    this->world = world;
    this->world->setState(4);
    this->searcher = searcher;
    this->explorer = new Explorer(6);
    this->plannPublisher = nh.advertise<std_msgs::Int16>("/control/goal", 1000);
    this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
    this->statesPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/states", 1000);
    this->heighLevelPlanPublisher = nh.advertise<std_msgs::String>("/model/heighLevelPlan", 1000);
    this->controlPlanPublisher = nh.advertise<std_msgs::String>("/model/controlLevelPlan", 1000);
    this->automaton = new ltl_Automaton();
    // this->automaton->create_automaton("!F(RC & X(RR))");
    // this->automaton->create_automaton("G(RC->F(CC))");
    this->automaton->create_automaton("RC");
    this->firstTime = true;
}

void planner::CreatePlan()
{
    if(this->world->getIsMapSet() && this->world->getStateIsSet())
    {
        this->world->Compute_Abstraction();
        this->searchedPlan->clearPlan();
        this->searchedPlan->startSearchCell = ((this->world->getHeight() / 2) - 1) * this->world->getWidth() + (this->world->getWidth() / 2);
        int goal;
        std::vector<int> *path;
        std::vector<std::tuple<std::string, int>> *plann;
        bool ltl_validation;
        std::string LTLFailedState;
        bool explorer_validation;
        int explorer_fail_state;
        SelectGoal goalSelector(this->world);
        goal = goalSelector.getGoal();
        do {
            path = this->searcher->getRute(this->searchedPlan->startSearchCell, goal, this->searchedPlan->nextInvalidCells);
            plann = this->world->getStatesChain(path);
            ltl_validation = this->automaton->evaluate_formula(plann, &LTLFailedState);
            // If fail, push just the valid states
            // Logg results for testing
            // this->test(path, plann, ltl_validation);  
            if(!ltl_validation)
            {
                ROS_INFO_STREAM("LTL fail");
                this->searchedPlan->invalidPLanFromCell(LTLFailedState, path, plann);
            }
            else
                ROS_INFO_STREAM("LTL succes");
            explorer_validation = this->explorer->simpleRouteValidation(this->world, path, &explorer_fail_state);
            if(!explorer_validation)
            {
                ROS_INFO_STREAM("EXPLORER fail ");
                ROS_INFO_STREAM(explorer_fail_state);
                this->searchedPlan->invalidPLanFromCell(explorer_fail_state, path, plann);
            }
            else ROS_INFO_STREAM("EXPLORER succes ");
            this->searchedPlan->pushPlan(path, plann);
        } while(!explorer_validation && !ltl_validation);
        this->searchedPlan->nextInvalidCells->clear();
        // Push current plan to explorer
        if(firstTime)
        {
            this->PublicPlann();
            firstTime = false;
        }
        //Public to Rviz
        this->PublicPath(this->world->getMap(), this->searchedPlan->path);
        this->PublicStates(this->world->getMap(), this->world->getMapStates());
        this->PublicAllPLan();
    }
}

void planner::ReadLaneState(const std_msgs::Float32MultiArray &loacalization_array)
{
    auto state = loacalization_array.data.at(0);
    // this->world->setState(state);
}

void planner::ReadMap(const nav_msgs::OccupancyGrid &map)
{
    this->world->setMap(map);
    this->searchedPlan->setWorldWitdh(this->world->getWidth());
}

void planner::ReadGoal(const std_msgs::Bool &isInGoal)
{
    // this->searchedPlan->moveForward();
    ROS_INFO_STREAM("PUBLISHING");
    PublicPlann();
}

void planner::PublicPlann()
{
    int next_signal = this->searchedPlan->getNextStep();
    this->world->setState(this->world->getState() + next_signal);
    std_msgs::Int16 controlMsg;
    controlMsg.data = next_signal;
    this->plannPublisher.publish(controlMsg);
}

void planner::PublicAllPLan()
{
    auto plan = this->searchedPlan->plann;
    
    std::stringstream heighLevelPlan;
    std_msgs::String heighLevelPlanMessage;

    std::stringstream controlPLan;
    std_msgs::String controlPLanMessage;

    for(auto state = plan->begin(); state != plan->end(); state++)
    {
        std::string stateName = std::get<0>(*state);
        heighLevelPlan << stateName << ",";

        int controlSignal = std::get<1>(*state);
        controlPLan << controlSignal << ",";
    }

    heighLevelPlanMessage.data = heighLevelPlan.str();
    this->heighLevelPlanPublisher.publish(heighLevelPlanMessage);

    controlPLanMessage.data = controlPLan.str();
    this->controlPlanPublisher.publish(controlPLanMessage);
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

void planner::test(const std::vector<int> *path, const std::vector<std::tuple<std::string, int>> *plann, bool ltlResult)
{
    ROS_INFO_STREAM("Logging results");

    // std::cout << "LTL machine reult: " << ltlResult << std::endl;

    // if(path == NULL)
    //     return;
    // std::cout << "Searched states:" << std::endl;
    // for(auto i = path->begin(); i != path->end(); i++)
    //     std::cout << *i << ", ";
    // std::cout << "" << std::endl;

    // if(plann == NULL)
    //     return;
    // std::cout << "Hiegh level plan:" << std::endl;
    // for(auto i = plann->begin(); i != plann->end(); i++)
    //      std::cout << "(" << std::get<0>(*i) << "," << std::get<1>(*i) << "), ";
    // std::cout << "" << std::endl;

    // std::cout << "================================" << std::endl;
    // char control;
    // std::cin >> control;
}