#include <ros/ros.h>
#include "./local_planner.cpp"

static const int RATE_HZ = 5;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planner");
    ROS_INFO_STREAM("local_planner node initialized");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    local_planner local = local_planner(nh);
    int img_height = 160;
    int img_width = 160;
    cv::Mat imagePaint = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 0));
    sensor_msgs::ImagePtr imgmsg;

    while (ros::ok())
    {
        ros::spinOnce();
        local.plot_polinomials(imagePaint);
        if (local.polynomial_exists())
              local.ackerman_control(imagePaint);
        imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagePaint).toImageMsg();
        local.pub_image.publish(imgmsg);
        loop_rate.sleep();
    }

    return 0;
    
}