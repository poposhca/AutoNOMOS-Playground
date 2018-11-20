# AutoNOMOS LTL planner
## On working project

This repository is made based on the original AutoNomos repository (https://github.com/EagleKnights/AutoNOMOS), this is a work in progress.

## Dependecies

This project needs the following dependecies to fully work:
* `Autonomos simulation` https://github.com/EagleKnights/EK_AutoNOMOS_Sim   The gazebo simulation for the Autonomos car.
* `Spot` https://spot.lrde.epita.fr/ The LTL platform to specify LTL formulas

## Project ROS packages

The source code is mainly consists of the following packeges
* The LTL input node. This node acts as an interface for the user to specify a plan to kickoff the entire system.
* Map node. This nodes reads form a laser (Lidar) topic to create an occupancy grid of the enviorment.
* Heigh level planner. This node takes the LTL sentence, the Ocupancy Grid from map and the States array from the camare to create a plan for the robot to follow.
* Control. This is the low level node that performce a simulation based on the car dynamics to help create a plan.
* Test (name will change. Node that helps performce code testing and contains the lounch files that runs the hole system.