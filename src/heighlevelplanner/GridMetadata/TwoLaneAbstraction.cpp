#include "TwoLaneAbstraction.h"

TwoLaneAbstraction::TwoLaneAbstraction()
{
    this->map = new nav_msgs::OccupancyGrid();
    this->mapset = false;
}

void TwoLaneAbstraction::setMap(const nav_msgs::OccupancyGrid& map)
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

void TwoLaneAbstraction::setState(const std_msgs::Float32MultiArray &laneState)
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

const std::vector<int>* TwoLaneAbstraction::getMap()
{
    auto mapCopy = new std::vector<int>;
    int number_cells = this->map->info.width * this->map->info.height;
    mapCopy->resize(number_cells);
    std::copy(this->map->data.begin(), this->map->data.end(), mapCopy->begin());
    return mapCopy;
}

bool TwoLaneAbstraction::getIsMapSet()
{
    return this->mapset;
}

int TwoLaneAbstraction::getWidth()
{
    return this->map->info.width;
}

int TwoLaneAbstraction::getHeight()
{
    return this->map->info.height;
}

float TwoLaneAbstraction::getResolution()
{
    return this->map->info.resolution;
}

std::vector<std::string>* TwoLaneAbstraction::getStatesChain(std::vector<int> *chain)
{
    if(chain == NULL)
        return NULL;
    if(chain->size() == 0) 
        return NULL;
    
    std::vector<std::string> *resultChain = new std::vector<std::string>();
    for(auto i = chain->begin(); i != chain->end(); i++)
    {
        auto state_metadata = this->metadata->at(*i);
        auto name = this->name_state[state_metadata.cell_state];
        resultChain->push_back(name);
    }
    return resultChain;
}

void TwoLaneAbstraction::Compute_Abstraction()
{
    //TODO: Des-hardcodear. Considero que siempre hay una misma distancia entre lineas de cada lado (ooops)
    int map_cells_regions = this->map->info.width / 4;
    int stupid_count = 0;
    for(auto i = 0; i != this->map->info.width * this->map->info.height; i++)
    {
        cellMetadata newMetadata;
        if(stupid_count <= map_cells_regions)
            newMetadata.cell_state = 3;
        if(stupid_count <= map_cells_regions * 2 && stupid_count > map_cells_regions)
            newMetadata.cell_state = 4;
        if(stupid_count <= map_cells_regions * 3 && stupid_count > map_cells_regions * 4)
            newMetadata.cell_state = 5;
        if(stupid_count == this->map->info.width - 1)
            stupid_count = 0;
        else
            stupid_count += 1;
        this->metadata->at(i) = newMetadata;
    }
}

void TwoLaneAbstraction::test()
{

    std::cout << "TESTING -----------" << std::endl;

    //Metadata
    std::cout << "============================================" << std::endl;
    std::cout << map->info.width << std::endl;
    std::cout << map->info.height << std::endl;
    std::cout << map->info.resolution << std::endl;
    std::cout << "============================================" << std::endl;

    //Grid info
    std::cout << "============================================" << std::endl;
    for(auto i = 0; i < this->map->data.size(); i++)
        std::cout << "{Value: " << this->map->data.at(i) << ", State: " << this->metadata->at(i).cell_state << "},";
    std::cout << "" << std::endl;
    std::cout << "============================================" << std::endl;

}