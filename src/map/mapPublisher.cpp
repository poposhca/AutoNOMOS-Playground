#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "MapGetters/mapping.h"
#include "MapGetters/lidarSensingMap.h"

#define RATE_HZ 5

using namespace std;

void PublicMap(const nav_msgs::OccupancyGrid* map, const ros::Publisher publisher)
{
    publisher.publish(*map);
}

void PrintMapInConsole(const nav_msgs::OccupancyGrid* map)
{
    ROS_INFO_STREAM("Imprimiendo en consola:");
    int i = 0;
    for(int row = 0; row < map->info.height; row++)
    {
        for(int column = 0; column < map->info.width; column++)
            cout << map->data[i++] << " ";
        cout << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MapNode");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);
    auto mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/map", 1000);
    mapping *myMap = new lidarSensingMap(nh);  
    ROS_INFO_STREAM("Map node up & running");
    while(ros::ok)
    {
        ros::spinOnce();
        auto map = myMap->GetMap();
        //PrintMapInConsole(map);
        PublicMap(map, mapPublisher);
    }

    return 0;
}

    //Este bloque lo tengo que separar
    /*
    int width;
    int height;
    string file;
    ros::NodeHandle nhParamas("~");
    nhParamas.param<int>("width", width, 0);
    nhParamas.param<int>("height", height, 0);
    nhParamas.param<string>("mapFile", file, "");
    if(width == 0 && height == 0)
    {
        cout << "Empty Map!!!" << endl;
        return -9;
    }
    //
    */