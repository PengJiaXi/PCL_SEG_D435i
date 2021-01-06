/*
 * PCL_segment.hpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Peng Jiaxi
 *   Institute: HUST, Intelligent Robotics Lab
 */

#ifndef PCL_SEGMENT_H
#define PCL_SEGMENT_H

#include <ros/ros.h>
#include "pcl_seg/d435i.h"
#include "pcl_seg/processPointClouds.h"
#include <pcl/visualization/cloud_viewer.h>
#include <visualization_msgs/Marker.h>
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace lidar_obstacle_detection;

//！相机模型类，切换相机时只需要更换模型类即可使用
using CameraModel = D435i;


namespace pcl_process
{

    class PointCloudSegment
    {
    public:
        explicit PointCloudSegment(ros::NodeHandle nh);
        ~PointCloudSegment();

    private:
        //! ROS
        ros::NodeHandle nodeHandle_;
        ros::Publisher groundPublisher_;
        ros::Publisher obstaclePublisher_;
        ros::Publisher originPublisher_;
        ros::Publisher imuPublisher_;
        ros::Publisher boxPublisher_;

        //！camera
        CameraModel camera;

        //创建输入的点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI;
        //pcl viewer
        pcl::visualization::PCLVisualizer::Ptr viewer;
        pcl::visualization::PCLVisualizer::Ptr viewer2;
        pcl::visualization::PCLVisualizer::Ptr viewer3;
        pcl::visualization::PCLVisualizer::Ptr viewer4;

        //! 读取ros参数
        bool readParameters();

        //! 初始化任务
        void init();
        void MyGetOBB (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ& min_point, pcl::PointXYZ& max_point, pcl::PointXYZ& position, Eigen::Matrix3f& rotational_matrix) const;
        void Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int clusterId, pcl::visualization::PCLVisualizer::Ptr &viewer);
        void ros_Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int clusterId);
        void cityBlock(ProcessPointClouds<pcl::PointXYZ> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, Eigen::Vector3f pose_data);
        void showCloudsInROS(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloudClusters);
        void showCloudsInPCL(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &filteredCloud,
            const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &segmentCloud,
            const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloudClusters);
    };
} // namespace pcl_process
#endif //PCL_SEGMENT_H
