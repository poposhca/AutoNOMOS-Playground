#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "MapGetters/mapping.h"
#include "MapGetters/simpleMapTesting.h"

using namespace std;

void PrintMapInConsole(const nav_msgs::OccupancyGrid& map)
{
    int i = 0;
    for(int row = 0; row < map.info.height; row++)
    {
        for(int column = 0; column < map.info.width; column++)
            cout << map.data[i++] << " ";
        cout << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ocupancy Map Node");

    int width;
    int height;
    string file;
    ros::NodeHandle nh("~");

    nh.param<int>("width", width, 0);
    nh.param<int>("height", height, 0);
    nh.param<string>("mapFile", file, "");
    if(width == 0 && height == 0)
    {
        cout << "Empty Map!!!" << endl;
        return -9;
    }

    mapping *myMap = new SimpleMap(width, height, file);
    auto map = myMap->GetMap();
    PrintMapInConsole(map);
    cout << "Ya tengo mapa, otra vez" << endl;

    /*
    cout << "Publicando mapa" << endl;
    while(ros::ok)
    {
        myMap.publicMap();
    }
    */

    return 0;
}