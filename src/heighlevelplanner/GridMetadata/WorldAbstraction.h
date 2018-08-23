#ifndef __WORLDABSTRACTION_H__
#define __WORLDABSTRACTION_H__

#include <iostream>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include "CellInfo.h"

class WorldAbstraction
{
private:
    nav_msgs::OccupancyGrid *map;
    float mapResolution;
    bool mapset;
    int mapWidth;
    int mapHeight;

public:
    WorldAbstraction();
    void setMap(const nav_msgs::OccupancyGrid&);
    
    void Tests();
};

#endif