#ifndef __WORLDABSTRACTION_H__
#define __WORLDABSTRACTION_H__

#include <nav_msgs/OccupancyGrid.h>

class WorldAbstraction
{
public:
    virtual void setMap(const nav_msgs::OccupancyGrid&) = 0;
};

#endif