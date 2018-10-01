#include "astar.h"
#define debugc

//Cell Comparison class

bool cellComparison::operator() (const cellSearchInfo* lhs, const cellSearchInfo* rhs)
{
    return rhs->f < lhs->f;
}

//A* class

astar::astar(WorldAbstraction *world)
{
    this->world = world;
    this->cellsCreated = new std::map<int, cellSearchInfo*>;
    this->closedSet = new std::map<int, cellSearchInfo*>;
    this->minHeap = new std::priority_queue<cellSearchInfo*, std::vector<cellSearchInfo*>, cellComparison>();
}

std::vector<int>* astar::getRute(int start, int goal)
{
    auto startCell = createCell(start, -1, goal, 0);
    this->minHeap->push(startCell);
    this->cellsCreated->emplace(startCell->index, startCell);

    cellSearchInfo *actualCell = NULL;
    auto map = this->world->getMap();
    while(this->minHeap->size() != 0)
    {
        //Select cell to expand and push to the path
        actualCell = this->minHeap->top();
        this->minHeap->pop();
        this->closedSet->emplace(actualCell->index, actualCell);
        #ifdef debug
        std::cout << "1) Expandir: " << actualCell->index << " con f:" << actualCell->f << std::endl;
        #endif

        //Goal reached
        if(actualCell->index == goal)
                return reconstruct_path(goal);

        //Expand neighbors
        #ifdef debugb
        std::cout << "2) Expandir vecinos " << std::endl;
        #endif
        auto cell_coordenates = getGridCoordenates(actualCell->index);
        int vi = std::get<0>(cell_coordenates);
        int vj = std::get<1>(cell_coordenates);
        #ifdef debugb
        std::cout << "3) Coordenadas " << vi << "," << vj << std::endl;
        #endif
        for(int j = -1; j <= 1; j++)
            for(int i = -1; i <= 1; i++)
            {
                //Actual cell cooredentares (i,j)
                int actual_vi = vi + i;
                int actual_vj = vj + j;
                #ifdef debugb
                std::cout << "4) Candidatos " << actual_vi << "," << actual_vj << std::endl;
                #endif
                //Validate coordenates are insede ranges
                bool IsCellWidthValid = 0 <= actual_vi && actual_vi  < this->world->getWidth();
                bool IsCellHeightValid = 0 <= actual_vj && actual_vj < this->world->getHeight();
                bool IsTheSameCell = actual_vi == vi && actual_vj == vj;
                if(IsCellWidthValid && IsCellHeightValid && !IsTheSameCell)
                {
                    int cellIndex = getIndexFromCoordentase(actual_vi, actual_vj);
                    bool cellAlreadyCheked = cellIsInVector(this->closedSet, cellIndex);
                    if(!cellAlreadyCheked)
                    {
                        float tmp_g = actualCell->g + getDistance(actualCell->index, cellIndex);
                        bool cellAlreadyCreated = cellIsInVector(this->cellsCreated, cellIndex);
                        //TODO: Make a variable for the probability!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        bool cellIsFree = map->at(cellIndex) <= 200;
                        #ifdef debug
                        std::cout << "5) Califica " << actual_vi << "," << actual_vj << " en celda " << cellIndex << " con p=" << map->at(cellIndex) << std::endl;
                        #endif
                        if(!cellAlreadyCreated && cellIsFree)
                        {
                            auto newCell = createCell(cellIndex, actualCell->index, goal, tmp_g);
                            this->cellsCreated->emplace(newCell->index, newCell);
                            this->minHeap->push(newCell);
                            #ifdef debug
                            std::cout << "6.1) To expand: " << newCell->index << "whit f: " << newCell->f << std::endl;
                            #endif
                        }
                        else if(cellAlreadyCreated)
                        {
                            auto neighbor_cell = this->cellsCreated->at(cellIndex);
                            bool better_path = tmp_g < neighbor_cell->g;
                            if(better_path && cellIsFree)
                            {
                                updateCell(neighbor_cell, tmp_g);
                                #ifdef debug
                                std::cout << "6.2) To expand: " << neighbor_cell.index << "whit f: " << neighbor_cell.f << std::endl;
                                #endif
                            }
                        }
                    }
                }
            }
    #ifdef debug
    char control;
    std::cin >> control;
    #endif
    }

    //Partial result
    return reconstruct_path(actualCell->index);
}

//Helper methods

std::vector<int>* astar::reconstruct_path(int actualCell)
{
    auto result_path = new std::vector<int>;
    auto cell = this->cellsCreated->at(actualCell);
    while(cell->fatherIndex != -1)
    {
        result_path->push_back(cell->index);
        cell = this->cellsCreated->at(cell->fatherIndex);
    }
    return result_path;
}

cellSearchInfo* astar::createCell(int value, int origin, int goal, float g_value)
{
    auto newCell = new cellSearchInfo;
    newCell->index = value;
    newCell->fatherIndex = origin;
    newCell->g = g_value;
    newCell->h = getDistance(value, goal);
    newCell->f = newCell->g + newCell->h;
    #ifdef debug
    std::cout << "  From: " << newCell->fatherIndex << std::endl;
    std::cout << "  Valor g(x) " << newCell->g << std::endl;
    std::cout << "  Valor h(x) " << newCell->h << std::endl;
    std::cout << "  Valor f(x) " << newCell->f << std::endl;
    #endif
    return newCell;
}

void astar::updateCell(cellSearchInfo* cell, float g_value)
{
    cell->g = g_value;
    cell->f = cell->g + cell->h;
}

std::tuple<int,int> astar::getGridCoordenates(int value)
{
    float i = value % this->world->getWidth();
    float j = value / this->world->getWidth();
    return std::make_tuple(i, j);
}

int astar::getIndexFromCoordentase(int i, int j)
{
    return j * (this->world->getWidth() - 1) + i;
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

bool astar::cellIsInVector(std::map<int, cellSearchInfo*> *cellVector, int cellIndex)
{
    auto cell = cellVector->find(cellIndex);
    if(cell == cellVector->end())
        return false;
    return true;
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

}