#ifndef __ASTARABSTRACTION_H__
#define __ASTARABSTRACTION_H__

#include <iostream>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include "WorldAbstraction.h"
#include "CellInfo.h"

class AStarAbstraction : public WorldAbstraction
{
private:
    nav_msgs::OccupancyGrid *map;
    float mapResolution;
    bool mapset;
    int mapWidth;
    int mapHeight;

public:
    AStarAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    void Test();
};

#endif