//
// Created by lucas on 03.04.23.
//
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    int a;
    a = msg->width;
    ROS_INFO("Received Point Cloud");
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_camera_listener");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber vision = nh.subscribe("/camera/depth/points", 1, callback);

    //sensor_msgs::PointCloud2::Ptr point_cloud_ptr = bag.begin()->instantiate<sensor_msgs::PointCloud2>();
    /*
    if (!true) //point_cloud_ptr
    {
        ROS_FATAL("invalid message in rosbag");
        return 1;
    }
     */

    // Give a bit of time to move_group to connect & cache transforms
    // works around sporadic tf extrapolation errors
    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(0.2);
    while (ros::ok())
    {
        //point_cloud_ptr->header.stamp = ros::Time::now();
        loop_rate.sleep();
    }


    return 0;
}