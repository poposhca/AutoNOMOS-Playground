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
        this->metadata = new std::vector<cellMetadata>(number_cells);
        this->mapset = true;
    }
    std::copy(map.data.begin(), map.data.end(), this->map->data.begin());
}

void AStarAbstraction::setState(const std_msgs::Float32MultiArray &laneState)
{
    //TODO: Checar y corregir si es necesario el codigo de Lalo
    // int estadoPrevio = this->actual_state;
    // float max=0;
    // for(int i = 0; i < NUM_STATES * STATE_WIDTH; i++) {
    //     if(laneState.data[i]>max) {
    //         max=laneState.data[i];
    //     }
    // }

    // int countStates=0;
    // int state = -1;
    // for(int i = NUM_STATES * STATE_WIDTH - 1; i >= 0; i--) {
    //     if(laneState.data[i]==max) {
    //         int temp_state = (int)floor(i / STATE_WIDTH);
    //         if (temp_state != state){
    //             state = temp_state;
    //             countStates++;
    //         }
    //     }
    // }

    // if (countStates==1)
    //     this->actual_state = state;
    // else
    //     this->actual_state = -1; // no se pudo determinar el estado, ya que hay mas de uno posible

    this->actual_state = 4;
    std::cout << "Estado actual: " << this->name_state[this->actual_state] << std::endl;
}

const std::vector<int>* AStarAbstraction::getMap()
{
    auto mapCopy = new std::vector<int>;
    int number_cells = this->map->info.width * this->map->info.height;
    mapCopy->resize(number_cells);
    std::copy(this->map->data.begin(), this->map->data.end(), mapCopy->begin());
    return mapCopy;
}

bool AStarAbstraction::getIsMapSet()
{
    return this->mapset;
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


void AStarAbstraction::Compute_Abstraction()
{
    //TODO: Des-hardcodear. Considero que siempre hay una misma distancia entre lineas de cada lado (ooops)
    int map_cell_radious = this->map->info.width / 2;
    int map_cells_regions = map_cell_radious / 4;
    int stupid_count = 0;
    for(auto i = 0; i != this->map->info.width * this->map->info.height; i++)
    {
        cellMetadata newMetadata;
        if(stupid_count <= map_cells_regions)
            newMetadata.cell_state = 3;
        if(stupid_count <= map_cells_regions * 3 && stupid_count > map_cells_regions)
            newMetadata.cell_state = 4;
        if(stupid_count <= map_cells_regions * 4 && stupid_count > map_cells_regions * 3)
        {
            newMetadata.cell_state = 5;
            stupid_count = -1;
        }
        this->metadata->at(i) = newMetadata;
        stupid_count += 1;
    }
}

void AStarAbstraction::Test()
{

    std::cout << "TESTING -----------" << std::endl;

    //Test grid info
    std::cout << "============================================" << std::endl;
    for(auto i = this->map->data.begin(); i != this->map->data.end(); i++)
        std::cout << *i << ", ";
    std::cout << "" << std::endl;
    std::cout << "============================================" << std::endl;

    //Test metadata
    // std::cout << "============================================" << std::endl;
    // std::cout << map->info.width << std::endl;
    // std::cout << map->info.height << std::endl;
    // std::cout << map->info.resolution << std::endl;
    // std::cout << "============================================" << std::endl;

    //Test getGridMethod
    std::cout << "============================================" << std::endl;
    auto map = this->getMap();
    for(auto i = map->begin(); i != map->end(); i++)
        std::cout << *i << ", ";
    std::cout << "" << std::endl;
    std::cout << "============================================" << std::endl;

}