#include "laser.h"

laser::laser()
{
    this->obstacleDistance = 0.0f;
    this->obstacleCell = 0;
}

void laser::Clear()
{
    this->obstacleDistance = 0.0f;
    this->obstacleCell = 0;
}

bool laser::HasCellIndex(int cell)
{
    auto it = this->laserCells.find(cell);
    return it != this->laserCells.end();
}

void laser::SetObstacle(float dinstance, int cell)
{
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
        if(distance > this->obstacleDistance)
        {
            float delta = occupancy_grid->data[cell] + GetProbability(distance);
            occupancy_grid->data[cell] = delta <= 100 ? delta : 100;
        }
    }
}

float laser::GetProbability(float distance)
{
    //Normal CDF
    float std = 0.5;
    float p = (1 + erf((distance - this->obstacleDistance) / (std * sqrt(2)))) / 2;
    std::cout << "Probabilidad: " << p * 100 << std::endl;
    return p * 100;
}