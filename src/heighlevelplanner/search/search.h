
#ifndef _RUTEEXPLORER_
#define _RUTEEXPLORER_

#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class ruteExplorer
{
public:
    virtual std::vector<int>* getRute(const nav_msgs::OccupancyGrid&, int, int) = 0;
    virtual void setMapMetadata(const nav_msgs::OccupancyGrid&) = 0;
    virtual bool haveMap() = 0;
};

#endif