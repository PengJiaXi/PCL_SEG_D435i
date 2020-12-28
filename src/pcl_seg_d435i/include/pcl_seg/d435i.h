#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <cstdlib>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "example.h"  

class D435i
{
public:
    // 相机数据是否更新
    bool isUpdated;
    //获取下一帧信息
    void updateNextFrame();
    //初始化相机
    void init();
    
    //输出数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointClouds();
    Eigen::Vector3f getPose();

    //静态成员函数，用作回调
    static void static_pipeCb(rs2::frame);
    //静态类指针
    static D435i* static_class_ptr;

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
    Eigen::Vector3f accelVector;   // 线性加速度向量

    // 真正处理回调任务的成员函数
    void _pipeCallback(rs2::frame);
    //将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr _trans2pcl();
};

class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts);
    float3 process_accel(rs2_vector accel_data);
    bool check_imu_is_supported();

};
