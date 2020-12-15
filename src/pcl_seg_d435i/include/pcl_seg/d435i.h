#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <cstdlib>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class D435i
{
public:
    //获取下一帧信息
    void updateNextFrame();
    //初始化相机
    void init();
    //输出数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointClouds();
    Eigen::Quaternion<float>* getPose();

private:
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr;     //点云
    Eigen::Quaternion<float>* quaternion;   // Eigen四元数姿态角

    //将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans2pcl();
};
