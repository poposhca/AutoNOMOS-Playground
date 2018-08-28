#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <iostream>
#include <cmath>
#include <queue>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include "search.h"
#include "../GridMetadata/WorldAbstraction.h"

struct cellSearchInfo
{
    //The real index in the occupancy grid
    int index;
    // The heuristic h(x) value to the goal
    float h;
};

class cellComparison
{
public:
    bool operator() (const cellSearchInfo& lhs, const cellSearchInfo& rhs);
};

class astar : public ruteExplorer
{
private:
    //Properties
    WorldAbstraction *world;
    std::priority_queue<cellSearchInfo, std::vector<cellSearchInfo>, cellComparison> *minHeap;
    //Methods
    cellSearchInfo& createCell(int value, int goal);
    float getDistance(int value, int goal);
    bool cellIsInVector(std::vector<cellSearchInfo> &cellVector, int cellIndex);
    bool cellIsInVector(std::vector<cellSearchInfo> &cellVector, const cellSearchInfo &cell);

public:
    astar(WorldAbstraction *world);
    void setMapMetadata(const nav_msgs::OccupancyGrid&);
    std::vector<int>* getRute(const nav_msgs::OccupancyGrid&, int, int);
    void Test();
};

#endif