
#ifndef _RUTEEXPLORER_
#define _RUTEEXPLORER_

#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class ruteExplorer
{
public:
    virtual std::vector<int>* getRute(const nav_msgs::OccupancyGrid&, int, int) = 0;
};

#endif