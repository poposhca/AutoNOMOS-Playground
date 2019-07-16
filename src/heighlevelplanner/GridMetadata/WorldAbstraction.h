#ifndef __WORLDABSTRACTION_H__
#define __WORLDABSTRACTION_H__

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <vector>

class WorldAbstraction
{
public:
    virtual void setMap(const nav_msgs::OccupancyGrid&) = 0;
    virtual void setState(const int laneStates) = 0;
    virtual int getState() = 0;
    virtual const std::vector<int>* getMap() = 0;
    virtual const std::vector<int>* getMapStates() = 0;
    virtual bool getIsMapSet() = 0;
    virtual bool getStateIsSet() = 0;
    virtual int getWidth() = 0;
    virtual int getHeight() = 0;
    virtual float getResolution() = 0;
    virtual std::vector<std::tuple<std::string, int>>* getStatesChain(std::vector<int> *chain) = 0;
    virtual void Compute_Abstraction() = 0;
};

#endif