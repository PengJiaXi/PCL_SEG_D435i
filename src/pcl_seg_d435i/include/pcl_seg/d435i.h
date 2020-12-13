#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <cstdlib>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class D435i
{
public:
    //获取转化为pcl格式的一帧点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getNextFrame();
    //初始化相机
    void init();

private:
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans2pcl();
};
