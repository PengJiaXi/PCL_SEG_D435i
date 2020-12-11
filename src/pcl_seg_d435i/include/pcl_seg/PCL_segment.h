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

#include "pcl_seg/processPointClouds.h"

using namespace lidar_obstacle_detection;

namespace pcl_process {

class PointCloudSegment{
public:
    explicit PointCloudSegment(ros::NodeHandle nh);
    ~PointCloudSegment();

private:
    //! ROS
    ros::NodeHandle nodeHandle_;
    ros::Publisher groundPublisher_;
    ros::Publisher obstaclePublisher_;

    //! 读取ros参数
    bool readParameters();

    //! 初始化任务
    void init();

    void cityBlock(ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud);
};
}
#endif //PCL_SEGMENT_H