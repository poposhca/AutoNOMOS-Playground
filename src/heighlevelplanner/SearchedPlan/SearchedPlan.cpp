#include "./SearchedPlan.h"
#include <iostream>

SearchedPlan::SearchedPlan()
{
    this->isPlanSet = false;
    this->startSearchCell = 0;
    this->goalSearchCell = 0;
    this->path = new std::vector<int>;
    this->nextInvalidCells = new std::vector<int>;
    this->plann = new std::vector<std::tuple<std::string, int>>;

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
    for(auto cell = path->begin(); cell != path->end(); cell++)
        this->path->push_back(*cell);
    for(auto state = plann->begin(); state != plann->end(); state++)
        this->plann->push_back(*state);
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
