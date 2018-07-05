#ifndef _MAPPINH_H_
#define _MAPPINH_H_

#include <nav_msgs/OccupancyGrid.h>

class mapping
{
public:
    virtual nav_msgs::OccupancyGrid* GetMap() = 0;
};

#endif