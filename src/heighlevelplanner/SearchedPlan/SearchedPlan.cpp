#include "./SearchedPlan.h"
#include <iostream>
#include <ros/ros.h>

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

    this->nextInvalidCells->push_back(path->at(planCellNUmber));
    for(int i = path->size() - 1; i != planCellNUmber; i--)
    {
        int value = path->at(i);
        path->pop_back();
    }
    path->pop_back();
}

void SearchedPlan::invalidPLanFromCell(int startCell, std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann)
{
    ROS_INFO_STREAM("Invalidating path");
    int CellNUmber = 0;
    std::vector<int>::iterator pathCellPosition;
    for(auto pathIt = path->begin(); pathIt != path->end(); pathIt++)
    {
        int state = *pathIt;
        if(startCell == state)
        {
            ROS_INFO_STREAM("FOUND");
            pathCellPosition = pathIt;
            break;
        }
        CellNUmber++;
    }
    path->erase(pathCellPosition, path->end());

    ROS_INFO_STREAM("Invalidating plan");
    for(int i = plann->size() - 1; i != CellNUmber; i--)
    {
        plann->pop_back();
    }
    ROS_INFO_STREAM("finish");
    
    this->nextInvalidCells->push_back(startCell);

}

void SearchedPlan::moveForward()
{
    this->path->erase(this->path->begin());
    this->plann->erase(this->plann->begin());
    for(auto cell = this->path->begin(); cell != this->path->end(); cell++)
    {
        int actual_cell_index = *cell;
        *cell = actual_cell_index - this->world_width;
    }
    this->startSearchCell = this->path->back();
}

int SearchedPlan::getNextStep()
{
    if(this->plann->empty())
        return 100;
    auto next_state = this->plann->at(0);
    int control_signal = std::get<1>(next_state);
    return control_signal;
}

void SearchedPlan::clearPlan()
{
    this->plann->clear();
    this->path->clear();
}
