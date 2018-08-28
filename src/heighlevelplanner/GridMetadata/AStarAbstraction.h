#ifndef __ASTARABSTRACTION_H__
#define __ASTARABSTRACTION_H__

#include <iostream>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include "WorldAbstraction.h"

class AStarAbstraction : public WorldAbstraction
{
private:
    nav_msgs::OccupancyGrid *map;
    bool mapset;

public:
    AStarAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    int getWidth();
    int getHeight();
    float getResolution();
    void Test();
};

#endif