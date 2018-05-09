#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <fstream>
#include <string>

using namespace std;
using namespace std_msgs;

class Map
{

private:
    UInt32 width;
    UInt32 height;
    Int8 *GridMap;

public:

    //Constructor creates map
    Map(int width, int height)
    {
        this->width.data = width;
        this->height.data = height;

        int vectorSize = width * height;
        cout << "Vector: " << vectorSize << endl;
        this->GridMap = new Int8[vectorSize];
        Int8 valueStruct;
        valueStruct.data = -1;
        for(int i = 0; i < vectorSize; i++)
        {
            this->GridMap[i] = valueStruct;
        }
    }

    void publicMap()
    {

    }

    void PrintMapInConsole()
    {
        int i = 0;
        for(int row = 0; row < this->height.data; row++)
        {
            for(int column = 0; column < this->width.data; column++)
                cout << this->GridMap[i++].data << " ";
            cout << endl;
        }
    }

    void LoadDummyMap(string file)
    {
        ifstream input(file);
        if(input.is_open())
        {
            int i = 0;
            string row;
            while(getline(input, row))
            {
                for(int j = 0; j < this->width.data; j++)
                    this->GridMap[i++].data = row[j] == '1' ? 100 : 0;
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

    Map myMap(width, height);
    myMap.LoadDummyMap(file);
    myMap.PrintMapInConsole();


    return 0;
}