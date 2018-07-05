#ifndef _SIMPLEMAP_H_
#define _SIMPLEMAP_H_

#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include "mapping.h"

using namespace std;

class SimpleMap : public mapping
{
private:
    nav_msgs::OccupancyGrid map;
    //No se si usar estos parametros
    unsigned int width;
    unsigned int height;
public:
    SimpleMap(int width, int height, string file);
    nav_msgs::OccupancyGrid* GetMap();
    void LoadDummyMap(string file);
};

#endif