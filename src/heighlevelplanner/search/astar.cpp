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

std::vector<int>* astar::getRute(int start, int goal)
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
        auto cell_coordenates = getGridCoordenates(actualCell.index);
        int vi = std::get<0>(cell_coordenates);
        int vj = std::get<1>(cell_coordenates);
        for(int j = -1; j <= 1; j++)
            for(int i = -1; i <= 1; i++)
            {
                //Actual cell cooredentares (i,j)
                int actual_vi = vi + i;
                int actual_vj = vj + j;

                //Validate coordenates are insede ranges
                bool IsCellWidthValid = 0 < actual_vi && actual_vi  < this->world->getWidth();
                bool IsCellHeightValid = 0 < actual_vj && actual_vj < this->world->getHeight();
                bool IsTheSameCell = actual_vi == vi && actual_vj == vj;
                if(IsCellWidthValid && IsCellHeightValid && !IsTheSameCell)
                {
                    int cellIndex = getIndexFromCoordentase(actual_vi, actual_vj);
                    bool cellAlreadyCheked = cellIsInVector(closedSet, cellIndex);
                    auto map = this->world->getMap();
                    bool cellIsFree = map->at(cellIndex) == 0;
                    if(!cellAlreadyCheked && cellIsFree)
                    {
                        auto newCell = createCell(cellIndex, goal);
                        this->minHeap->push(newCell);
                        closedSet.push_back(newCell);
                    }
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

std::tuple<int,int> astar::getGridCoordenates(int value)
{
    float i = value % this->world->getWidth();
    float j = value / this->world->getWidth();
    return std::make_tuple(i, j);
}

int astar::getIndexFromCoordentase(int i, int j)
{
    return i * this->world->getWidth() + j;
}

float astar::getDistance(int value, int goal)
{
    float m = this->world->getResolution() / 2;

    //Ubicacion de la celda actual
    auto value_coordenates = getGridCoordenates(value);
    float value_i_m = (float)std::get<0>(value_coordenates) * this->world->getResolution() + m;;
    float value_j_m = (float)std::get<1>(value_coordenates) * this->world->getResolution() + m;;

    //Ubicacion de la celda objetivo
    auto goal_coordenates = getGridCoordenates(goal);
    float goal_i_m = (float)std::get<0>(goal_coordenates) * this->world->getResolution() + m;;
    float goal_j_m = (float)std::get<1>(goal_coordenates) * this->world->getResolution() + m;;

    return sqrt(pow(value_i_m - goal_i_m, 2) + pow(value_j_m - goal_j_m, 2));
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