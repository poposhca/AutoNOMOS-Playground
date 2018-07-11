#ifndef _LIDARSENSING_H_
#define _LIDARSENSING_H_

#include <tuple>
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "mapping.h"
#include "../laser/laser.h"

class lidarSensingMap : public mapping
{
private:

    //Variables
    //ROS laser subscriber
    ros::Subscriber sub_laser_scan;
    //Prameter variables
    float laser_range_min;        // minimum range value [m]
    float laser_range_max;        // maximum range value [m]
    int angle_offset;
    float cell_resolution;
    //Laser grid
    std::vector<laser> *lasersGrid;
    nav_msgs::OccupancyGrid *occupancy_grid;
    float cells_number_by_row;
    float laser_ranges[360];

    //Methods
    void InitOccupancyGrid();
    void InitLasersPreProcessing();
    void UpdateLidar(const sensor_msgs::LaserScan &msg);
    std::tuple<int, int> MapCoordenatesToGridSpace(float x, float y);
    int GridCoordentaesToCellNumber(int grid_x, int grid_y);
    void WriteProbabilityOnGrid(int grid_x, int gird_y, int p);

    void GetBeamIntersectionCells(std::vector<std::tuple<int,int>> *valid_laser_beams);

public:

    lidarSensingMap(ros::NodeHandle nh);
    nav_msgs::OccupancyGrid* GetMap();
};

#endif