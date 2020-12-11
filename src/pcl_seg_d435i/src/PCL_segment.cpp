/*
 * PCL_segment.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Peng Jiaxi
 *   Institute: HUST, Intelligent Robotics Lab
 */

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_seg/processPointClouds.h"
#include "pcl_seg/PCL_segment.h"

// using templates so also include .cpp to help linkern
#include "processPointClouds.cpp"

using namespace lidar_obstacle_detection;

namespace pcl_process
{

    void PointCloudSegment::cityBlock(ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
    {
        // ----------------------------------------------------
        // -----Open 3D viewer and display City Block     -----
        // ----------------------------------------------------

        // Setting hyper parameters

        // FilterCloud
        float filterRes = 0.4;

        //Eigen是一个C++线性代数运算库
        Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
        Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
        // SegmentPlane
        int maxIterations = 40;
        float distanceThreshold = 0.3;
        // Clustering
        float clusterTolerance = 0.5;
        int minsize = 10;
        int maxsize = 140;

        // First:Filter cloud to reduce amount of points
        // 过滤点云，保留指定感兴趣立方体内的点，并去除自身车顶的点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint, maxpoint);

        // Second: Segment the filtered cloud into obstacles and road
        // 执行Ransac，获得一对数据，first为地面外的点，second为地面内的点
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
            filteredCloud, maxIterations, distanceThreshold);

        // 发布地面点云信息
        sensor_msgs::PointCloud2 ground_point_cloud;
        pcl::toROSMsg(*(segmentCloud.second),ground_point_cloud);
        ground_point_cloud.header.frame_id = "ground_clouds";       
        groundPublisher_.publish(ground_point_cloud);
        
        // 发布障碍物点云信息
        sensor_msgs::PointCloud2 obstacle_point_cloud;
        pcl::toROSMsg(*segmentCloud.first, obstacle_point_cloud);
        obstacle_point_cloud.header.frame_id = "obstacle_clouds";
        obstaclePublisher_.publish(obstacle_point_cloud);

        //    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        //    renderPointCloud(viewer,inputCloud,"inputCloud");

        // Third: Cluster different obstacle cloud
        // 获得每个聚类的点云序列
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
                                                                                                               clusterTolerance,
                                                                                                               minsize, maxsize);
        int clusterId = 0;

        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {

            std::cout << "cluster size";
            pointProcessorI->numPoints(cluster); // 控制台显示点云尺寸

            // 显示每类障碍物点云
            // Fourth: Find bounding boxes for each obstacle cluster
            // 绘制边界框
            ++clusterId;
        }
    }

    PointCloudSegment::PointCloudSegment(ros::NodeHandle nh)
    {
        this->nodeHandle_ = nh;

        if (!readParameters())
        {
            ros::requestShutdown();
        }

        init();
    }

    PointCloudSegment::~PointCloudSegment(){}

    void PointCloudSegment::init()
    {
        std::cout << "starting enviroment" << std::endl;

        //初始化Subscriber和Publisher
        std::string groundPubTopicName;
        int groundPubQueueSize;
        bool groundPubLatch;
        std::string obstacleTopicName;
        int obstacleQueueSize;
        bool obstacleLatch;

        nodeHandle_.param("publishers/ground/topic", groundPubTopicName, std::string("ground_point_cloud"));
        nodeHandle_.param("publishers/ground/queue_size", groundPubQueueSize, 1);
        nodeHandle_.param("publishers/ground/latch", groundPubLatch, false);
        nodeHandle_.param("publishers/obstacle/topic", obstacleTopicName, std::string("obstacle_point_cloud"));
        nodeHandle_.param("publishers/obstacle/queue_size", obstacleQueueSize, 1);
        nodeHandle_.param("publishers/obstacle/latch", obstacleLatch, false);

        groundPublisher_ =
            nodeHandle_.advertise<sensor_msgs::PointCloud2>(groundPubTopicName, groundPubQueueSize, groundPubLatch);
        obstaclePublisher_ =
            nodeHandle_.advertise<sensor_msgs::PointCloud2>(obstacleTopicName, obstacleQueueSize, obstacleLatch);

        //声明一个XYZI类的点云处理对象
        ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

        //获取PCD文件目录下的所有文件路径，排序并组成一个vector
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/pjx/cleaner_ws/src/pcl_seg/data/pcd/data_1");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI; //创建输入的点云对象

        while (ros::ok())
        {
            // Clear viewer
            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlock(pointProcessorI, inputCloudI); //障碍物检测
            streamIterator++;                        //下一个文件
            if (streamIterator == stream.end())
            {
                streamIterator = stream.begin();
            }
            ros::spinOnce();
        }
    }

    bool PointCloudSegment::readParameters()
    {
        // Load common parameters.
        //   nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
        //   nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
        //   nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

        //   // Check if Xserver is running on Linux.
        //   if (XOpenDisplay(NULL)) {
        //     // Do nothing!
        //     ROS_INFO("[YoloObjectDetector] Xserver is running.");
        //   } else {
        //     ROS_INFO("[YoloObjectDetector] Xserver is not running.");
        //     viewImage_ = false;
        //   }

        //   // Set vector sizes.
        //   nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));
        //   numClasses_ = classLabels_.size();
        //   rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
        //   rosBoxCounter_ = std::vector<int>(numClasses_);

        return true;
    }

} // namespace pcl_process