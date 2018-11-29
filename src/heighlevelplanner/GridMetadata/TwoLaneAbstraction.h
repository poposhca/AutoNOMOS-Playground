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

public:
    TwoLaneAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    void setState(const std_msgs::Float32MultiArray &laneState);
    const std::vector<int>* getMap();
    bool getIsMapSet();
    int getWidth();
    int getHeight();
    float getResolution();
    std::vector<std::tuple<std::string, int>>* getStatesChain(std::vector<int> *chain);
    void Compute_Abstraction();
    void test();
};

#endif