#ifndef _ASTAR_
#define _ASTAR_

#include <queue>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include "search.h"

struct CellInfo_Past
{
    //The real index in the occupancy grid
    int index;
    // The heuristic h(x) value to the goal
    float h;
    //TODO: Add lane info data
};

class cellComparison
{
public:
    bool operator() (const CellInfo_Past& lhs, const CellInfo_Past& rhs);
};

class astar : public ruteExplorer
{
private:
    //Properties
    bool mapset;
    float mapResolution;
    int mapWidth;
    int mapHeight;
    std::priority_queue<CellInfo_Past, std::vector<CellInfo_Past>, cellComparison> *minHeap;
    //Methods
    CellInfo_Past& createCell(int value, int goal);
    float getDistance(int value, int goal);
    bool cellIsInVector(std::vector<CellInfo_Past> &cellVector, int cellIndex);
    bool cellIsInVector(std::vector<CellInfo_Past> &cellVector, const CellInfo_Past &cell);

public:
    astar();
    bool haveMap();
    void setMapMetadata(const nav_msgs::OccupancyGrid&);
    std::vector<int>* getRute(const nav_msgs::OccupancyGrid&, int, int);

    void Test();
};

#endif