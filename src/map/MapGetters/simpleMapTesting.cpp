#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <string>
#include <vector>
#include "simpleMapTesting.h"

using namespace std;
using namespace std_msgs;


//Constructor creates map
SimpleMap::SimpleMap(int width, int height, string file)
{
    
    this->width = width;
    this->height = height;

    LoadDummyMap(file);

    geometry_msgs::Point originPoint;
    originPoint.x = 0;
    originPoint.y = 0;
    originPoint.z = 0;

    geometry_msgs::Quaternion originQuarternion;
    originQuarternion.x = 0;
    originQuarternion.y = 0;
    originQuarternion.z = 0;
    originQuarternion.w = 0;

    nav_msgs::MapMetaData metadata;
    metadata.resolution = 1.0;
    metadata.width = this->width;
    metadata.height = this->height;

    geometry_msgs::Pose originPose;
    originPose.position = originPoint;
    originPose.orientation = originQuarternion;
    metadata.origin = originPose;
    map.info = metadata;
}

nav_msgs::OccupancyGrid* SimpleMap::GetMap()
{
    //return this->map;
    return NULL;
}

void SimpleMap::LoadDummyMap(string file)
{
    ifstream input(file);
    if(input.is_open())
    {
        string row;
        while(getline(input, row))
        {
            for(int j = 0; j < this->width; j++)
                this->map.data.push_back(row[j] == '1' ? 100 : 0);
        }
    }
    else
    {
        cout << "Problem with file: " << file << endl;
    }
}
