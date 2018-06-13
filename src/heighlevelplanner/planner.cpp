#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "search/search.h"
#include "search/astar.h"

using namespace std;

class planner
{
private:
    ruteExplorer *plannerSearcher;
    ros::Publisher pathPublisher;
    bool test;

public:

    planner()
    {
        this->plannerSearcher = new astar();
        ros::NodeHandle nh;
        this->pathPublisher = nh.advertise<nav_msgs::OccupancyGrid>("model/path", 1000);
        this->test = true;
    }

    void ReadMap(const nav_msgs::OccupancyGrid &map)
    {
        if(test)
        {
        cout << "Leyendo mapa de " << map.info.width << " x " << map.info.height << endl;
        if(!this->plannerSearcher->haveMap())
        {
            cout << "Seteando metedata" << endl;
            this->plannerSearcher->setMapMetadata(map);
        }
        cout << "Obteniendo ruta" << endl;
        auto path = this->plannerSearcher->getRute(map, 0,28);
        for(auto i = path->begin(); i != path->end(); i++)
            cout << *i << "->";
        //testing();
        cout << "EOL" << endl;
        test = false;
        }
    }

    void PublicPath(nav_msgs::OccupancyGrid &map, const std::vector<int> *path)
    {
        auto grid = map.data;
        for(auto i = path->begin(); i != path->end(); i++)
            map.data[*i] = 50;
        this->pathPublisher.publish(map);
    }

    void testing()
    {
        (dynamic_cast<astar*>(this->plannerSearcher))->Test();
    }
};


void ExecuteROS(int argc, char **argv, planner *plan)
{
    ros::init(argc, argv, "Planner_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("model/map", 1000, &planner::ReadMap, plan);
    ros::spin();
}

int main(int argc, char **argv)
{
    planner *p = new planner;
    ExecuteROS(argc, argv, p);
    //p->testing();
    return 0;
}