#ifndef _ASTAR_
#define _ASTAR_

#include <queue>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include "search.h"

struct CellInfo
{
    //The real index in the occupancy grid
    int index;
    // The heuristic h(x) value to the goal
    float h;
};

class cellComparison
{
public:
    bool operator() (const CellInfo& lhs, const CellInfo& rhs);
};

class astar : public ruteExplorer
{
private:
    //Properties
    bool mapset;
    float mapResolution;
    int mapWidth;
    int mapHeight;
    std::priority_queue<CellInfo, std::vector<CellInfo>, cellComparison> *minHeap;
    //Methods
    CellInfo& createCell(int value, int goal);
    float getDistance(int value, int goal);
    bool cellIsInVector(std::vector<CellInfo> &cellVector, int cellIndex);
    bool cellIsInVector(std::vector<CellInfo> &cellVector, const CellInfo &cell);

public:
    astar();
    bool haveMap();
    void setMapMetadata(const nav_msgs::OccupancyGrid&);
    std::vector<int>* getRute(const nav_msgs::OccupancyGrid&, int, int);

    void Test();
};

#endif