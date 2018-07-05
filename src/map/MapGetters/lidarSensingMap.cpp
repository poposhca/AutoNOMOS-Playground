#include <iostream>
#include <algorithm>
#include "lidarSensingMap.h"

void lidarSensingMap::UpdateLidar(const sensor_msgs::LaserScan &msg)
{
    std::copy(msg.ranges.begin(), msg.ranges.end(), this->laser_ranges);
}

nav_msgs::OccupancyGrid* lidarSensingMap::GetMap()
{
    ROS_INFO_STREAM("Creando Metadata");

    //Updates gird
    nav_msgs::OccupancyGrid& actual_grid = *this->occupancy_grid;
    fill(actual_grid.data.begin(), actual_grid.data.end(), -1);
    ROS_INFO_STREAM("Grid con " << actual_grid.data.size() << " datos");

    //Header
    //TODO: Guardar la secuancia que se crea
    actual_grid.header.seq = 0;
    actual_grid.header.stamp = ros::Time::now();
    actual_grid.header.frame_id = "0";

    //Metadata
    actual_grid.info.resolution = this->cell_resolution;
    actual_grid.info.width = this->cells_number_by_row;
    actual_grid.info.height = this->cells_number_by_row;
    actual_grid.info.map_load_time = ros::Time::now();

    ROS_INFO_STREAM("Llenando Grid");
    float x, y, h;
	int x_grid, y_grid;
	for (int i = 0; i < 360; ++i)
	{
		h = laser_ranges[i];
		if (h <= laser_range_max && h > 2 * cell_resolution) // the value is in range 
		{
			ROS_INFO_STREAM("laser_ranges["<< i << "] = " << h);
			
			x = h * cos( (i + angle_offset) * M_PI / 180);
			y = h * sin( (i + angle_offset) * M_PI / 180);

            ROS_INFO_STREAM("point: ( " << x << ", " << y << " )");
			
			x_grid = (int) (x * cells_number_by_row / laser_range_max / 2 + cells_number_by_row / 2);
			y_grid = (int) (y * cells_number_by_row / laser_range_max / 2 + cells_number_by_row / 2);

            ROS_INFO_STREAM("point: ( " << x_grid << ", " << y_grid << " )");

            //La probabilidad no se como llenarla bien
			// int prob = laser_grid.data[x_grid + y_grid * grid_width];
			// laser_grid.data[x_grid + y_grid * grid_width] = prob == 100 ? 100 : prob++ ;
			actual_grid.data[x_grid + y_grid * cells_number_by_row] = 99;
		}
	}

    ROS_INFO_STREAM("Regresando grid");

    return this->occupancy_grid;
}

lidarSensingMap::lidarSensingMap(ros::NodeHandle nh)
{
    // Init params
    ros::NodeHandle priv_nh("~");
    std::string node_name = ros::this_node::getName();
    priv_nh.param<float>(node_name+"/laser_range_min", laser_range_min, .008);
    priv_nh.param<float>(node_name+"/laser_range_max", laser_range_max, 6);
    priv_nh.param<float>(node_name+"/cell_resolution", cell_resolution, 0.25);
    priv_nh.param<int>(node_name+"/angle_offset", angle_offset, 0);

    //Init OccupancyGrid
    this->occupancy_grid = new nav_msgs::OccupancyGrid;
    this->cells_number_by_row = (2 * laser_range_max) / cell_resolution;
    this->occupancy_grid->data.resize(cells_number_by_row * cells_number_by_row);

    //Subscriber (lidar listener)
    this->sub_laser_scan = nh.subscribe("/scan", 1, &lidarSensingMap::UpdateLidar, this);
}