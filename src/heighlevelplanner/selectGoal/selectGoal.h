#ifndef __SELECT_GOAL_H__
#define __SELECT_GOAL_H__

#include <vector>
#include "../GridMetadata/WorldAbstraction.h"

class SelectGoal 
{
private:
    WorldAbstraction *world;
    std::vector<int> *slectedCells;
public:
    SelectGoal(WorldAbstraction *world);
    int getGoal();
};

#endif