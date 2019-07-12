#include "./SearchedPlan.h"
#include <iostream>

SearchedPlan::SearchedPlan()
{
    this->world_width = 0;
    this->isPlanSet = false;
    this->startSearchCell = 0;
    this->goalSearchCell = 0;
    this->path = new std::vector<int>;
    this->nextInvalidCells = new std::vector<int>;
    this->plann = new std::vector<std::tuple<std::string, int>>;

}

void SearchedPlan::setWorldWitdh(int witdh)
{
    this->world_width = witdh;
}

void SearchedPlan::setPlan(std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann)
{
    this->path->resize(path->size());
    std::copy(path->begin(), path->end(), this->path->begin());
    this->plann->resize(plann->size());
    std::copy(plann->begin(), plann->end(), this->plann->begin());
    this->isPlanSet = true;
}

void SearchedPlan::pushPlan(std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann)
{
    if(path == NULL || path == NULL)
        return;
    if(path->empty() || path->empty())
        return;
    if(!this->isPlanSet) this->isPlanSet = true;
    for(auto cell = path->begin(); cell != path->end(); cell++)
        this->path->push_back(*cell);
    for(auto state = plann->begin(); state != plann->end(); state++)
        this->plann->push_back(*state);
    this->startSearchCell = this->path->back();
}

void SearchedPlan::invalidPLanFromCell(std::string startCell, std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann)
{
    int planCellNUmber = 0;
    std::vector<std::tuple<std::string, int>>::iterator planCellPosition;
    for(auto planIt = plann->begin(); planIt != plann->end(); planIt++)
    {
        auto state = std::get<0>(*planIt);
        if(state == startCell)
        {
            planCellPosition = planIt;
            break;
        }
        planCellNUmber++;
    }
    plann->erase(planCellPosition, plann->end());

    for(int i = path->size() - 1; i != planCellNUmber; i--)
    {
        int value = path->at(i);
        path->pop_back();
        nextInvalidCells->push_back(value);
    }

    this->startSearchCell = path->at(planCellNUmber);
}

void SearchedPlan::moveForward()
{
    this->path->erase(this->path->begin());
    for(auto cell = this->path->begin(); cell != this->path->end(); cell++)
    {
        int actual_cell_index = *cell;
        *cell = actual_cell_index - this->world_width;
    }
}

int SearchedPlan::getNextStep()
{
    std::cout << "Getting next" << std::endl;
    auto next_state = this->plann->at(0);
    int control_signal = std::get<1>(next_state);
    std::cout << "Next signal" << control_signal << std::endl;
    return control_signal;
}