#include "selectGoal.h"

SelectGoal::SelectGoal(WorldAbstraction *world)
{
    this->world = world;
    this->slectedCells = new std::vector<int>;
}

int SelectGoal::getGoal()
{
    const std::vector<int>* grid = this->world->getMap();
    int width = this->world->getWidth();
    int height = this->world->getHeight();
    int lastCell = (width * height) - 1;
    for(int i = lastCell; i >= 0; i--)
    {
        int cellContent = grid->at(i);
        if(cellContent < 50)
        {
            return i;
        }
    }
}