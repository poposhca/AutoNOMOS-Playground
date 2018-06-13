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

using namespace std;
using namespace std_msgs;

class Map
{

private:
    ros::Publisher mapPublisher;
    nav_msgs::OccupancyGrid map;

    //No se si usar estos parametros
    unsigned int width;
    unsigned int height;
    
public:

    //Constructor creates map
    Map(int width, int height, string file)
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

        ros::NodeHandle nh;
        mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/map", 1000);
    }

    void publicMap()
    {
        mapPublisher.publish(map);
    }

    void PrintMapInConsole()
    {
        int i = 0;
        for(int row = 0; row < this->height; row++)
        {
            for(int column = 0; column < this->width; column++)
                cout << this->map.data[i++] << " ";
            cout << endl;
        }
    }

    void LoadDummyMap(string file)
    {
        ifstream input(file);
        if(input.is_open())
        {
            string row;
            while(getline(input, row))
            {
                cout << row << endl;
                for(int j = 0; j < this->width; j++)
                    this->map.data.push_back(row[j] == '1' ? 100 : 0);
            }
        }
        else
        {
            cout << "Problem with file: " << file << endl;
        }
    }

};

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

    Map myMap(width, height, file);
    myMap.PrintMapInConsole();

    cout << "Publicando mapa" << endl;
    while(ros::ok)
    {
        myMap.publicMap();
    }

    return 0;
}