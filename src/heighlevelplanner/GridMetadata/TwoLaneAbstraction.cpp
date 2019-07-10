#include "TwoLaneAbstraction.h"

TwoLaneAbstraction::TwoLaneAbstraction()
{
    this->map = new nav_msgs::OccupancyGrid();
    this->mapset = false;
    this->stateSet = false;
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

void TwoLaneAbstraction::setState(const int laneStates)
{
    if(!this->stateSet)
        this->stateSet = true;
    this->actual_state = laneStates;
}

//Calculation hardcoded for Gazebo simulatuion
void TwoLaneAbstraction::Compute_Abstraction()
{
    int actualState = this->actual_state;
    std::cout << "WITH:" << this->map->info.width << std::endl;
    for(int i = 0; i < this->map->info.height; i++)
    {
        for(int j = 0; j < this->map->info.width; j++)
        {
            int cell = (this->map->info.width*i)+j;
            int state = 0;
            if(j == 28) state = actualState - 2;
            if(j == 29) state = actualState - 1;
            if(j == 30) state = actualState;
            if(j == 31) state = actualState + 1;
            if(j == 32) state = actualState + 2;
            cellMetadata newMetadata;
            newMetadata.cell_state = state ;
            this->metadata->at(cell) = newMetadata;
        }
    }
}

bool TwoLaneAbstraction::getStateIsSet()
{
    return this->stateSet;
}

const std::vector<int>* TwoLaneAbstraction::getMap()
{
    auto mapCopy = new std::vector<int>;
    int number_cells = this->map->info.width * this->map->info.height;
    mapCopy->resize(number_cells);
    std::copy(this->map->data.begin(), this->map->data.end(), mapCopy->begin());
    return mapCopy;
}

const std::vector<int>* TwoLaneAbstraction::getMapStates()
{
    auto statesVector = new std::vector<int>;
    for(auto cellMetadata = this->metadata->begin(); cellMetadata != this->metadata->end(); cellMetadata++)
        statesVector->push_back(cellMetadata->cell_state);
    return statesVector;
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

std::vector<std::tuple<std::string, int>>* TwoLaneAbstraction::getStatesChain(std::vector<int> *chain)
{
    if(chain == NULL)
        return NULL;
    if(chain->size() == 0) 
        return NULL;
    
    auto *resultChain = new std::vector<std::tuple<std::string, int>>;
    for(auto actual_state = chain->begin(); actual_state != chain->end(); actual_state++)
    {
        //Get actual state name
        auto state_metadata = this->metadata->at(*actual_state);
        auto name = this->name_state[state_metadata.cell_state];
        //Get next state index if existis
        auto next_state_ptr = actual_state + 1;
        int next_state = state_metadata.cell_state;
        if(next_state_ptr != chain->end()) 
            next_state = this->metadata->at(*next_state_ptr).cell_state;
        //Create and set new state tuple <state name, control signal>
        auto control_signal = getControlSignal(state_metadata.cell_state, next_state);
        auto newStateTuple = std::make_tuple(name, control_signal);
        resultChain->push_back(newStateTuple);
    }
    return resultChain;
}

int TwoLaneAbstraction::getControlSignal(int actual_state, int next_state)
{
    if(actual_state == next_state) 
        return 0;
    if(actual_state > next_state) 
        return 1;
    if(actual_state < next_state) 
        return -1;
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