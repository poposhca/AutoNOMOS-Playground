#ifndef __TwoLaneAbstraction_H__
#define __TwoLaneAbstraction_H__

#include <tuple>
#include <iostream>
#include <algorithm>
#include "WorldAbstraction.h"

class TwoLaneAbstraction : public WorldAbstraction
{
private:
    struct cellMetadata
    {
        //The most probable state for the cell
        int cell_state;
    };
    std::string name_state[7] = {"OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};
    int getControlSignal(int actual_state, int next_state);
    nav_msgs::OccupancyGrid *map;
    std::vector<cellMetadata> *metadata;
    int actual_state;
    bool mapset;
    bool stateSet;

public:
    TwoLaneAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    void setState(const int laneStates);
    const std::vector<int>* getMap();
    const std::vector<int>* getMapStates();
    bool getIsMapSet();
    bool getStateIsSet();
    int getWidth();
    int getHeight();
    int getState();
    float getResolution();
    std::vector<std::tuple<std::string, int>>* getStatesChain(std::vector<int> *chain);
    void Compute_Abstraction();
    void test();
};

#endif