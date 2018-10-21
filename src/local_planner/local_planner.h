#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"

static const uint32_t MY_ROS_QUEUE_SIZE = 100;
static const double PI = 3.14159265;

static const int NUM_STATES = 7;
static const int STATE_WIDTH = 20;
// static const int STATE_WIDTH_PIX = 22;

class local_planner 
{

private:
    //ROS
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Publisher pub_speed_sta;
    ros::Subscriber sub_localization;
    ros::Subscriber sub_des_state;
    ros::Subscriber sub_ransac_left;
    ros::Subscriber sub_ransac_center;
    ros::Subscriber sub_ransac_right;
    ros::Subscriber sub_pts_left;
    ros::Subscriber sub_pts_center;
    ros::Subscriber sub_pts_right;
    //Planner constants
    std::string nombre_estado [NUM_STATES] = { "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR"};
    std::string controlador;
    double integralPID;
    double prevErrorPID;
    int estado_actual;
    int estado_deseado;
    int car_center;
    int car_speed;
    double min_steering;
    double max_steering;
    int image_height;
    double Kp;
    double Ki;
    double Kd;
    double Kdist;
    double Kh;
    int state_width_pix;
    bool polyDetectedLeft;
    bool polyDetectedCenter;
    bool polyDetectedRight;
    int R;
    int C;
    int L;
    nav_msgs::GridCells arr_left;
    nav_msgs::GridCells arr_center;
    nav_msgs::GridCells arr_right;
    int car_text_position;
    //Helper methods
    void get_pts_left(const nav_msgs::GridCells& array);
    void get_pts_center(const nav_msgs::GridCells& array);
    void get_pts_right(const nav_msgs::GridCells& array);
    void get_ransac_left(const nav_msgs::GridCells& poly);
    void get_ransac_center(const nav_msgs::GridCells& poly);
    void get_ransac_right(const nav_msgs::GridCells& poly);
    void get_localization(const std_msgs::Float32MultiArray& locArray);
    void get_ctrl_desired_state(const std_msgs::Int16& val);
    bool ackerman_control_next_points(double y_next_dist, cv::Point& pt_car, cv::Point& y_next_pt, cv::Point& y_next_pt2);
    double PID(double error, double dt, double Kp, double Ki, double Kd);

public:
    ros::Publisher pub_image;
    local_planner(ros::NodeHandle nh);
    void ackerman_control(cv::Mat& image);
    void plot_polinomials(cv::Mat& image);
    bool polynomial_exists();

};
