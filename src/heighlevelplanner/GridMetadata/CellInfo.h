#ifndef __CELLINFO_H__
#define __CELLINFO_H__

struct CellInfo
{
    //The real index in the occupancy grid
    int index;
    // The heuristic h(x) value to the goal
    float h;
    //TODO: Check if int is the correct type
    int region;
};

#endif