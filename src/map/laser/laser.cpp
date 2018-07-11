#include "laser.h"

laser::laser()
{
    this->isIntersectingObstacle = false;
    this->obstacleDistance = 0.0f;
    this->obstacleCell = 0;
}

void laser::Clear()
{
    this->isIntersectingObstacle = false;
    this->obstacleDistance = 0.0f;
}

bool laser::HasCellIndex(int cell)
{
    auto it = this->laserCells.find(cell);
    return it != this->laserCells.end();
}

void laser::SetObstacle(float dinstance, int cell)
{
    this->isIntersectingObstacle = true;
    this->obstacleDistance = dinstance;
    this->obstacleCell = cell;
}

void laser::insertCell(int cellIndex, float distanceFromOrigin)
{
    auto pair = std::pair<int, float>(cellIndex, distanceFromOrigin);
    this->laserCells.insert(pair);
}

void laser::WriteProbabilityOnGrid(nav_msgs::OccupancyGrid *occupancy_grid)
{
    for(auto i = this->laserCells.begin(); i != this->laserCells.end(); i++)
    {
        int cell = i->first;
        float distance = i->second;
        if(distance > this->obstacleDistance) occupancy_grid->data[cell] = 99;
    }
}

bool laser::IsIntersectingObstacle()
{
    return this->isIntersectingObstacle;
}