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
        float newProbability = 0;
        //Obtener nueva probabilidad
        if(this->obstacleDistance == 0)
            newProbability = occupancy_grid->data[cell] - GetProbability(0);
        else if(distance < this->obstacleDistance)
            newProbability = occupancy_grid->data[cell] - GetProbability(distance);
        else
            newProbability = occupancy_grid->data[cell] + GetProbability(distance);
        //Checar cotas superior e inferior
        if(newProbability < 0) 
            newProbability = 0;
        if(newProbability > 100) 
            newProbability = 100;
#ifdef DEBUG
        std::cout << "Nueva probabilidad: " << newProbability << std::endl;
#endif
        occupancy_grid->data[cell] = newProbability;
    }
}

float laser::GetProbability(float distance)
{
    //Normal CDF
    float std = 0.5;
    float p = (1 + erf((distance - this->obstacleDistance) / (std * sqrt(2)))) / 2;
    return p * 100;
}