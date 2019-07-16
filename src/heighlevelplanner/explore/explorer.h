#ifndef __EXPLORER_H__
#define __EXPLORER_H__

#include <string>
#include <vector>
#include <tuple>
#include "../GridMetadata/WorldAbstraction.h"

class Explorer
{
private:
    int saveDistance;
public:
    Explorer(int saveDistance);
    bool simpleRouteValidation(WorldAbstraction *world, std::vector<int> *path, int *failState);
};

#endif