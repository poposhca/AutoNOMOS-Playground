#ifndef _LASER_H_
#define _LASER_H_

#include <map>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>

class laser
{
private:
    std::map<int, float> laserCells;
    bool isIntersectingObstacle;
    float obstacleDistance;
    int obstacleCell;
public:
    void Clear();
    bool HasCellIndex(int cell);
    void SetObstacle(float dinstance, int cell);
    void insertCell(int cellIndex, float distanceFromOrigin);
    void WriteProbabilityOnGrid(nav_msgs::OccupancyGrid *occupancy_grid);
    bool IsIntersectingObstacle();
    laser();
};

#endif