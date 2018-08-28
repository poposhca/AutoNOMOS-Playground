#include "AStarAbstraction.h"

AStarAbstraction::AStarAbstraction()
{
    this->map = new nav_msgs::OccupancyGrid();
    this->mapset = false;
}

void AStarAbstraction::setMap(const nav_msgs::OccupancyGrid& map)
{
    if(!this->mapset)
    {
        int number_cells = map.info.width * map.info.height;
        this->map->data.resize(number_cells);
        this->map->info = map.info;
        this->map->header = map.header;
        this->mapset = true;
    }
    std::copy(map.data.begin(), map.data.end(), this->map->data.begin());
}

int AStarAbstraction::getWidth()
{
    return this->map->info.width;
}

int AStarAbstraction::getHeight()
{
    return this->map->info.height;
}

float AStarAbstraction::getResolution()
{
    return this->map->info.resolution;
}

void AStarAbstraction::Test()
{

    std::cout << "TESTING -----------" << std::endl;

    //Test grid info
    // for(auto i = this->map->data.begin(); i != this->map->data.end(); i++)
    //     std::cout << *i << ", ";
    // std::cout << "" << std::endl;

    //Test metadata
    // std::cout << "============================================" << std::endl;
    // std::cout << map->info.width << std::endl;
    // std::cout << map->info.height << std::endl;
    // std::cout << map->info.resolution << std::endl;
    // std::cout << "============================================" << std::endl;

}