//#define DEBUGLASER
#include "lidarSensingMap.h"

lidarSensingMap::lidarSensingMap(ros::NodeHandle nh)
{
    ros::NodeHandle priv_nh("~");
    std::string node_name = ros::this_node::getName();
    priv_nh.param<float>(node_name+"/laser_range_min", laser_range_min, 0.15);
    priv_nh.param<float>(node_name+"/laser_range_max", laser_range_max, 6);
    priv_nh.param<float>(node_name+"/cell_resolution", cell_resolution, 0.25);
    priv_nh.param<int>(node_name+"/angle_offset", angle_offset, 90);
    InitOccupancyGrid();
    ROS_INFO_STREAM("Iniciando Pre-procesamiento:");
    InitLasersPreProcessing();
    this->sub_laser_scan = nh.subscribe("/scan", 1, &lidarSensingMap::UpdateLidar, this);
}

void lidarSensingMap::InitOccupancyGrid()
{
    this->occupancy_grid = new nav_msgs::OccupancyGrid;
    this->cells_number_by_row = (2 * laser_range_max) / cell_resolution;
    this->occupancy_grid->data.resize(cells_number_by_row * cells_number_by_row);
    fill(this->occupancy_grid->data.begin(), this->occupancy_grid->data.end(), 50);
    this->occupancy_grid->info.origin.position.x = 0;
    this->occupancy_grid->info.origin.position.y = 0;
    this->occupancy_grid->info.origin.position.z = 0;
    this->occupancy_grid->info.origin.orientation.x = 0;
    this->occupancy_grid->info.origin.orientation.y = 0;
    this->occupancy_grid->info.origin.orientation.z = 0;
    this->occupancy_grid->info.origin.orientation.w = 0;
    this->occupancy_grid->info.resolution = this->cell_resolution;
    this->occupancy_grid->info.width = this->cells_number_by_row;
    this->occupancy_grid->info.height = this->cells_number_by_row;
}

void lidarSensingMap::InitLasersPreProcessing()
{
    this->lasersGrid = new std::vector<laser>;
    float delta = this->cell_resolution / 2;
    for(int i = 0; i < 360; i++)
    {
#ifdef DEBUGLASER
        ROS_INFO_STREAM("Laser: " << i);
#endif
        laser newLaser;
        float x = cos((i + angle_offset) * M_PI / 180);
        float y = sin((i + angle_offset) * M_PI / 180);
        float l = delta;
        while(l < this->laser_range_max)
        {
            float space_x = l * x;
            float space_y = l * y;
            auto gird_coordenates = MapCoordenatesToGridSpace(space_x, space_y);
            auto gridCell = GridCoordentaesToCellNumber(std::get<0>(gird_coordenates), std::get<1>(gird_coordenates));
            if(!newLaser.HasCellIndex(gridCell))
            {
                newLaser.insertCell(gridCell, l);
            }
            l += delta; 
        }
        this->lasersGrid->push_back(newLaser);
    }
}

void lidarSensingMap::UpdateLidar(const sensor_msgs::LaserScan &msg)
{
    std::copy(msg.ranges.begin(), msg.ranges.end(), this->laser_ranges);
}

nav_msgs::OccupancyGrid* lidarSensingMap::GetMap()
{
#ifdef DEBUG
    ROS_INFO_STREAM("Creando Metadata");
#endif
    //Updates gird
    nav_msgs::OccupancyGrid& actual_grid = *this->occupancy_grid;

    //Header
    //TODO: Guardar la secuancia que se crea
    actual_grid.header.seq = 0;
    actual_grid.header.stamp = ros::Time::now();
    actual_grid.header.frame_id = "0";
    actual_grid.info.map_load_time = ros::Time::now();

    int gridCell;
    float x, y, h;
    std::tuple<int, int> gird_coordenates;
	for (int i = 0; i < 360; ++i)
	{
        auto actual_laser = this->lasersGrid->at(i);
		h = laser_ranges[i];
		if (h <= laser_range_max && h > laser_range_min)
		{
			x = h * cos( (i + angle_offset) * M_PI / 180);
			y = h * sin( (i + angle_offset) * M_PI / 180);
			gird_coordenates = MapCoordenatesToGridSpace(x,y);
            gridCell = GridCoordentaesToCellNumber(std::get<0>(gird_coordenates), std::get<1>(gird_coordenates));
            actual_laser.SetObstacle(h, gridCell);
		}
        else
            actual_laser.Clear();
        actual_laser.WriteProbabilityOnGrid(this->occupancy_grid);
#ifdef DEBUGLASER
        ROS_INFO_STREAM("Datos del laser:");
        std::cout << i << "->" << this->laser_ranges[i] << std::endl;
        std::cout << "Punto en el espacio: ( " << x << ", " << y << " )" << std::endl;
        std::cout << "Grid point: ( " << std::get<0>(gird_coordenates) << ", " << std::get<1>(gird_coordenates) << " )" << std::endl;
        std::cout << "Celada del grid: " << gridCell << std::endl;
#endif
	}
    return this->occupancy_grid;
}

std::tuple<int,int> lidarSensingMap::MapCoordenatesToGridSpace(float x, float y)
{
    int x_grid = (int) ((this->cells_number_by_row / 2.0f) + (x / this->cell_resolution)); 
    int y_grid = (int) ((this->cells_number_by_row / 2.0f) + (y / this->cell_resolution)); 
    return std::make_tuple(x_grid, y_grid); 
}

int lidarSensingMap::GridCoordentaesToCellNumber(int grid_x, int grid_y)
{
    return grid_x +  grid_y * this->cells_number_by_row;
}