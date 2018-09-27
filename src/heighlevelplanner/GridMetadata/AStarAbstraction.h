#ifndef __ASTARABSTRACTION_H__
#define __ASTARABSTRACTION_H__

#include <iostream>
#include <algorithm>
#include <string>
#include "WorldAbstraction.h"

class AStarAbstraction : public WorldAbstraction
{
private:
    struct cellMetadata
    {
        //The most probable state for the cell
        int cell_state;
    };
    std::string name_state [7] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};
    nav_msgs::OccupancyGrid *map;
    std::vector<cellMetadata> *metadata;
    int actual_state;
    bool mapset;

public:
    AStarAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    void setState(const std_msgs::Float32MultiArray &laneState);
    const std::vector<int>* getMap();
    bool getIsMapSet();
    int getWidth();
    int getHeight();
    float getResolution();
    virtual void Compute_Abstraction();
    void test();
};

#endif