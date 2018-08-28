#include "astar.h"

//Cell Comparison class

bool cellComparison::operator() (const cellSearchInfo& lhs, const cellSearchInfo& rhs)
{
    return rhs.h < lhs.h;
}

//A* class

astar::astar(WorldAbstraction *world)
{
    this->world = world;
    this->minHeap = new std::priority_queue<cellSearchInfo, std::vector<cellSearchInfo>, cellComparison>();
}

std::vector<int>* astar::getRute(const nav_msgs::OccupancyGrid& actualMap, int start, int goal)
{
    std::vector<int> *resultPath = new std::vector<int>;

    //Cells already evaluated
    std::vector<cellSearchInfo> closedSet;

    auto startCell = createCell(start, goal);
    this->minHeap->push(startCell);
    closedSet.push_back(startCell);

    cellSearchInfo actualCell;
    while(this->minHeap->size() > 0)
    {
        //Select cell to expand and push to the path
        actualCell = this->minHeap->top();
        this->minHeap->pop();
        resultPath->push_back(actualCell.index);
        if(actualCell.index == goal)
                return resultPath;

        //Expand neighbors
        //TODO: DES-HARDCODEAR, ESTO ES EL QUE ROMPE EL CODIGO EN EJECUCION!!!
        for(int i = -4; i <= 4; i++)
        {
            int cellIndex = actualCell.index + i;
            bool cellAlreadyCheked = cellIsInVector(closedSet, cellIndex);
            bool cellIsFree = actualMap.data[cellIndex] == 0;
            if(!cellAlreadyCheked && cellIsFree)
            {
                auto newCell = createCell(cellIndex, goal);
                this->minHeap->push(newCell);
                closedSet.push_back(newCell);
            }
        }
    }

    return resultPath;
}

cellSearchInfo& astar::createCell(int value, int goal)
{
    cellSearchInfo newCell;
    newCell.index = value;
    newCell.h = getDistance(value, goal);
    cellSearchInfo &cellRef = newCell;
    return cellRef;
}

float astar::getDistance(int value, int goal)
{
    float m = this->world->getResolution() / 2;

    //Ubicacion de la celda actual
    float value_i = value % this->world->getWidth();
    float value_j = value / this->world->getWidth();
    float value_i_m = (float)value_i * this->world->getResolution() + m;
    float value_j_m = (float)value_j * this->world->getResolution() + m;

    //Ubicacion de la celda objetivo
    int goal_i = goal % this->world->getWidth();
    int goal_j = goal / this->world->getWidth();
    float goal_i_m = (float)goal_i * this->world->getResolution() + m;
    float goal_j_m = (float)goal_j * this->world->getResolution() + m;

    return sqrt(pow(value_i - goal_i_m, 2) + pow(value_j - goal_j_m, 2));
}

bool astar::cellIsInVector(std::vector<cellSearchInfo> &cellVector, int cellIndex)
{
    for(auto i = cellVector.begin(); i != cellVector.end(); i++)
    {
        auto actualCell = *i;
        if(actualCell.index == cellIndex) return true;
    }
    return false;
}

bool astar::cellIsInVector(std::vector<cellSearchInfo> &cellVector, const cellSearchInfo &cell)
{
    for(auto i = cellVector.begin(); i != cellVector.end(); i++)
    {
        auto actualCell = *i;
        if(actualCell.index == cell.index) return true;
    }
    return false;
}

//Internal testing method

void astar::Test()
{
    std::cout << this->world->getResolution() << "," << this->world->getHeight() << "," << this->world->getWidth() << std::endl;
    
    cellSearchInfo c1;
    c1.index = 15;

    cellSearchInfo c2;
    c2.index = 5;

    cellSearchInfo c3;
    c3.index = 10;

    cellSearchInfo c4;
    c3.index = 2;

    std::vector<cellSearchInfo> vect;
    vect.push_back(c1);
    vect.push_back(c2);
    vect.push_back(c3);

    bool ans;
    ans = cellIsInVector(vect, c4);
    //FALSE answer
    std::cout << (ans ? "true" : "false") << std::endl;

    //TRUE answer
    ans = cellIsInVector(vect, c3);
    std::cout << (ans ? "true" : "false") << std::endl;

}