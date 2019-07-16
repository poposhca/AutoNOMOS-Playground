#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <tuple>
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>
#include <map>
#include <nav_msgs/OccupancyGrid.h>
#include "search.h"
#include "../GridMetadata/WorldAbstraction.h"

struct cellSearchInfo
{
    //The real index in the occupancy grid
    int index;
    //Index from the origin cell in the path, the same as index if the cell is the starting point
    int fatherIndex;
    //Cost to move from start to this cell following the generated path
    float g;
    // The heuristic h(x) value to the goal
    float h;
    //Total cost
    float f;
};

class cellComparison
{
public:
    bool operator() (const cellSearchInfo* lhs, const cellSearchInfo* rhs);
};

class astar : public ruteExplorer
{
private:
    //Properties
    int *lastGoalFound;
    WorldAbstraction *world;
    std::map<int, cellSearchInfo*> *cellsCreated;
    std::map<int, cellSearchInfo*> *closedSet;
    std::priority_queue<cellSearchInfo*, std::vector<cellSearchInfo*>, cellComparison> *minHeap;
    //Main methods
    bool validRouteExists(int start, int goal, int* outStart);
    std::vector<int>* search(int start, int goal);
    std::vector<int>* reconstruct_path(int actualCell);
    //Helper methods
    cellSearchInfo* createCell(int value, int origin, int goal, float g_value);
    void updateCell(cellSearchInfo* cell, float g_value);
    bool cellIsInVector(std::map<int, cellSearchInfo*> *cellVector, int cellIndex);
    float getDistance(int value, int goal);
    std::tuple<int,int> getGridCoordenates(int value);
    int getIndexFromCoordentase(int i, int j);

public:
    astar(WorldAbstraction *world);
    void setMapMetadata(const nav_msgs::OccupancyGrid&);
    std::vector<int>* getRute(int start, int goal, std::vector<int> *preCLosedSet);
    void Test();
};

#endif