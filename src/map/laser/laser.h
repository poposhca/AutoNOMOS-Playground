#ifndef _LASER_H_
#define _LASER_H_

#include <map>
#include <iostream>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>

class laser
{
private:
    //The grid graph
    std::map<int, float> laserCells;
    //Properties setted if there is an obstacle
    float obstacleDistance;
    int obstacleCell;
public:
    laser();
    void Clear();
    bool HasCellIndex(int cell);
    void SetObstacle(float dinstance, int cell);
    void insertCell(int cellIndex, float distanceFromOrigin);
    void WriteProbabilityOnGrid(nav_msgs::OccupancyGrid *occupancy_grid);
    float GetProbability(float distance);
};

#endif