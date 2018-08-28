#ifndef __WORLDABSTRACTION_H__
#define __WORLDABSTRACTION_H__

#include <nav_msgs/OccupancyGrid.h>

class WorldAbstraction
{
public:
    virtual void setMap(const nav_msgs::OccupancyGrid&) = 0;
    virtual int getWidth() = 0;
    virtual int getHeight() = 0;
    virtual float getResolution() = 0;
};

#endif