#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <tuple>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "GridMetadata/WorldAbstraction.h"
#include "selectGoal/selectGoal.h"
#include "search/search.h"
#include "explore/explorer.h"
#include "ltlAtomaton/ltlAutomaton.h"
#include "SearchedPlan/SearchedPlan.h"

#define NUM_STATES 7
#define STATE_WIDTH 20

class planner
{
private:
    SearchedPlan *searchedPlan;
    WorldAbstraction *world;
    ltl_Automaton *automaton;
    ruteExplorer *searcher;
    Explorer *explorer;
    ros::Publisher plannPublisher;
    ros::Publisher pathPublisher;
    ros::Publisher statesPublisher;
    void PublicPlann();
    void PublicPath(const std::vector<int> *map, const std::vector<int> *path);
    void PublicStates(const std::vector<int> *map, const std::vector<int> *states);
    bool firstTime;

public:
    planner(WorldAbstraction *world, ruteExplorer *searcher);
    void ReadLaneState(const std_msgs::Float32MultiArray &loacalization_array);
    void ReadMap(const nav_msgs::OccupancyGrid &map);
    void ReadGoal(const std_msgs::Bool &isInGoal);
    void CreatePlan();
    void test(const std::vector<int> *path, const std::vector<std::tuple<std::string, int>> *plann, bool);
};

#endif