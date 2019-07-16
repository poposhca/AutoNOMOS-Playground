#include "./explorer.h"
#include <ros/ros.h>

Explorer::Explorer(int saveDistance)
{
    this->saveDistance = saveDistance;
}

bool Explorer::simpleRouteValidation(WorldAbstraction *world, std::vector<int> *path, int *failState)
{
    auto map = world->getMap();
    for(auto state = path->begin(); state != path->end(); state++)
    {
        int cell = *state + world->getWidth();
        int distance = 1;
        while(cell < map->size())
        {
            auto map_state = map->at(cell);
            if(map_state >= 50 && distance < saveDistance)
            {
                *failState = *state;
                return false;
            }
            else if(map_state >= 50 && distance >= saveDistance)
                break;
            cell += world->getWidth();
            distance++;
        }
    }
    return true;
}