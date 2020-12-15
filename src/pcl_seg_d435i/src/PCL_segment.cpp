/*
 * PCL_segment.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Peng Jiaxi
 *   Institute: HUST, Intelligent Robotics Lab
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_seg/processPointClouds.h"
#include "pcl_seg/PCL_segment.h"
#include "pcl_seg/d435i.h"

//#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <string.h>
// using templates so also include .cpp to help linkern
#include "processPointClouds.cpp"
#define MAXBOXS 1
using namespace lidar_obstacle_detection;
//PCL
//visualization
//pcl::visualization::PCLVisualizer viewer;
//boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewers(new pcl::visualization::PCLVisualizer("viewer"));
namespace pcl_process
{
    //     void PointCloudSegment::ros_Box(ros::NodeHandle nh,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int clusterId)
    //     {
    //         Eigen::Vector4f pcaCentroid;
    //         pcl::compute3DCentroid(*cloud, pcaCentroid);
    //         Eigen::Matrix3f covariance;
    //         pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    //         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    //         Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    //         Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    //         eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
    //         eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    //         eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    //         // std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
    //         // std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
    //         // std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
    //         /*
    //         // 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    //         pcl::PCA<pcl::PointXYZ> pca;
    //         pca.setInputCloud(cloudSegmented);
    //         pca.project(*cloudSegmented, *cloudPCAprojection);
    //         std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
    //         std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
    //         */
    //         Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    //         Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    //         tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
    //         tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
    //         tm_inv = tm.inverse();

    //         // std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
    //         // std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

    //         pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //         pcl::transformPointCloud(*cloud, *transformedCloud, tm);

    //         pcl::PointXYZ min_p1, max_p1;   //点云的最大值与最小值点
    //         Eigen::Vector3f c1, c;
    //         pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    //         c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

    //         std::cout << "型心c1(3x1):\n" << c1 << std::endl;

    //         Eigen::Affine3f tm_inv_aff(tm_inv);
    //         pcl::transformPoint(c1, c, tm_inv_aff);

    //         Eigen::Vector3f whd, whd1;
    //         whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    //         whd = whd1;
    //         float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

    //         std::cout << "width1=" << whd1(0) << std::endl;
    //         std::cout << "heght1=" << whd1(1) << std::endl;
    //         std::cout << "depth1=" << whd1(2) << std::endl;
    //         std::cout << "scale1=" << sc1 << std::endl;

    //         const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
    //         const Eigen::Vector3f    bboxT1(c1);

    //         const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
    //         const Eigen::Vector3f    bboxT(c);

    //         //变换到原点的点云主方向
    //         pcl::PointXYZ op;
    //         op.x = 0.0;
    //         op.y = 0.0;
    //         op.z = 0.0;
    //         Eigen::Vector3f px, py, pz;
    //         Eigen::Affine3f tm_aff(tm);
    //         pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
    //         pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
    //         pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
    //         pcl::PointXYZ pcaX;
    //         pcaX.x = sc1 * px(0);
    //         pcaX.y = sc1 * px(1);
    //         pcaX.z = sc1 * px(2);
    //         pcl::PointXYZ pcaY;
    //         pcaY.x = sc1 * py(0);
    //         pcaY.y = sc1 * py(1);
    //         pcaY.z = sc1 * py(2);
    //         pcl::PointXYZ pcaZ;
    //         pcaZ.x = sc1 * pz(0);
    //         pcaZ.y = sc1 * pz(1);
    //         pcaZ.z = sc1 * pz(2);

    // /////////////////////////////////////////////////////////////////////////////////
    //         ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //         visualization_msgs::Marker line_list;

    //         line_list.header.frame_id = "obstacle_clouds"+std::to_string(clusterId);
    //         line_list.lifetime = ros::Duration(1);
    //         line_list.ns = "lines";
    //         line_list.action = visualization_msgs::Marker::ADD;
    //         line_list.pose.orientation.w = 1.0;
    //         line_list.id = 2;
    //         line_list.type = visualization_msgs::Marker::LINE_LIST;
    //         line_list.scale.x = 0.01;
    //         // Line list is red
    //         line_list.color.r = 1.0;
    //         line_list.color.a = 1.0;
    //         geometry_msgs::Point p;
    //         p.x = c1(0)-0.5*whd1(0);
    //         p.y = c1(1)-0.5*whd1(1);
    //         p.z = c1(2)-0.5*whd1(2);
    //         geometry_msgs::Point p1;
    //         p1.x = c1(0)+0.5*whd1(0);
    //         p1.y = c1(1)-0.5*whd1(1);
    //         p1.z = c1(2)-0.5*whd1(2);
    //         geometry_msgs::Point p2;
    //         p2.x = c1(0)+0.5*whd1(0);
    //         p2.y = c1(1)+0.5*whd1(1);
    //         p2.z = c1(2)-0.5*whd1(2);
    //         geometry_msgs::Point p3;
    //         p3.x = c1(0)-0.5*whd1(0);
    //         p3.y = c1(1)+0.5*whd1(1);
    //         p3.z = c1(2)-0.5*whd1(2);
    //         geometry_msgs::Point p4;
    //         p4.x = c1(0)-0.5*whd1(0);
    //         p4.y = c1(1)-0.5*whd1(1);
    //         p4.z = c1(2)+0.5*whd1(2);
    //         geometry_msgs::Point p5;
    //         p5.x = c1(0)+0.5*whd1(0);
    //         p5.y = c1(1)-0.5*whd1(1);
    //         p5.z = c1(2)+0.5*whd1(2);
    //         geometry_msgs::Point p6;
    //         p6.x = c1(0)+0.5*whd1(0);
    //         p6.y = c1(1)+0.5*whd1(1);
    //         p6.z = c1(2)+0.5*whd1(2);
    //         geometry_msgs::Point p7;
    //         p7.x = c1(0)-0.5*whd1(0);
    //         p7.y = c1(1)+0.5*whd1(1);
    //         p7.z = c1(2)-0.5*whd1(2);
    //         line_list.points.push_back(p);
    //         line_list.points.push_back(p1);
    //         line_list.points.push_back(p1);
    //         line_list.points.push_back(p2);
    //         line_list.points.push_back(p2);
    //         line_list.points.push_back(p3);
    //         line_list.points.push_back(p3);
    //         line_list.points.push_back(p);

    //         line_list.points.push_back(p4);
    //         line_list.points.push_back(p5);
    //         line_list.points.push_back(p5);
    //         line_list.points.push_back(p6);
    //         line_list.points.push_back(p6);
    //         line_list.points.push_back(p7);
    //         line_list.points.push_back(p7);
    //         line_list.points.push_back(p4);

    //         line_list.points.push_back(p4);
    //         line_list.points.push_back(p);
    //         line_list.points.push_back(p5);
    //         line_list.points.push_back(p1);
    //         line_list.points.push_back(p6);
    //         line_list.points.push_back(p2);
    //         line_list.points.push_back(p7);
    //         line_list.points.push_back(p3);

    //         marker_pub.publish(line_list);
    //         line_list.header.stamp = ros::Time::now();
    //         ros::spinOnce();
    // //         while (ros::ok()) {
    // //     marker_pub.publish(line_list);
    // //     line_list.header.stamp = ros::Time::now();

    // //   }

    //     }
    void PointCloudSegment::Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int clusterId, pcl::visualization::PCLVisualizer::Ptr &viewer)
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::io::loadPCDFile("15967bda.pcd", *cloud);

        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

        // std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
        // std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
        // std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
        /*
        // 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloudSegmented);
        pca.project(*cloudSegmented, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
        */
        Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
        tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();                                     //R.
        tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>()); //  -R*t
        tm_inv = tm.inverse();

        // std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
        // std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformedCloud, tm);

        pcl::PointXYZ min_p1, max_p1; //点云的最大值与最小值点
        Eigen::Vector3f c1, c;
        pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
        c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

        // std::cout << "型心c1(3x1):\n" << c1 << std::endl;

        Eigen::Affine3f tm_inv_aff(tm_inv);
        pcl::transformPoint(c1, c, tm_inv_aff);

        Eigen::Vector3f whd, whd1;
        whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
        whd = whd1;
        float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; //点云平均尺度，用于设置主方向箭头大小

        std::cout << "width1=" << whd1(0) << std::endl;
        std::cout << "heght1=" << whd1(1) << std::endl;
        std::cout << "depth1=" << whd1(2) << std::endl;
        std::cout << "scale1=" << sc1 << std::endl;

        const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
        const Eigen::Vector3f bboxT1(c1);

        const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
        const Eigen::Vector3f bboxT(c);

        //变换到原点的点云主方向
        pcl::PointXYZ op;
        op.x = 0.0;
        op.y = 0.0;
        op.z = 0.0;
        Eigen::Vector3f px, py, pz;
        Eigen::Affine3f tm_aff(tm);
        pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
        pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
        pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
        pcl::PointXYZ pcaX;
        pcaX.x = sc1 * px(0);
        pcaX.y = sc1 * px(1);
        pcaX.z = sc1 * px(2);
        pcl::PointXYZ pcaY;
        pcaY.x = sc1 * py(0);
        pcaY.y = sc1 * py(1);
        pcaY.z = sc1 * py(2);
        pcl::PointXYZ pcaZ;
        pcaZ.x = sc1 * pz(0);
        pcaZ.y = sc1 * pz(1);
        pcaZ.z = sc1 * pz(2);

        std::string cube = "box" + std::to_string(clusterId);
        viewer->addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
        //viewer->addCoordinateSystem(0.5f*sc1);
        viewer->setBackgroundColor(0.0, 0.0, 0.0);
        viewer->spinOnce();

        viewer2->addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), cube);
        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cube);
        viewer2->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
        //viewer->addCoordinateSystem(0.5f*sc1);
        viewer2->setBackgroundColor(0.0, 0.0, 0.0);
        viewer2->spinOnce();
    }

    void PointCloudSegment::cityBlock(ProcessPointClouds<pcl::PointXYZ> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, Eigen::Vector3f pose_data)
    {
        // ----------------------------------------------------
        // -----Open 3D viewer and display City Block     -----
        // ----------------------------------------------------

        // Setting hyper parameters

        float voxelsize = 0.05;
        // SegmentPlane
        int maxIterations = 40;
        float distanceThreshold = 0.05;
        // Clustering
        float clusterTolerance = 0.05;
        int minsize = 10;
        int maxsize = 10000;

        // First:Filter cloud to reduce amount of points
        // 过滤点云，保留指定感兴趣立方体内的点，并去除自身车顶的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, voxelsize);

        // Second: Segment the filtered cloud into obstacles and road
        // 执行Ransac，获得一对数据，first为地面外的点，second为地面内的点
        // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
        //     filteredCloud, maxIterations, distanceThreshold);
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = 
            pointProcessorI->RansacSegmentPlaneWithPose(filteredCloud, maxIterations, distanceThreshold, pose_data);

        // 发布地面点云信息
        sensor_msgs::PointCloud2 ground_point_cloud;
        pcl::toROSMsg(*(segmentCloud.second), ground_point_cloud);
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
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
                                                                                                              clusterTolerance,
                                                                                                              minsize, maxsize);
        int clusterId = 0;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(inputCloudI, 255, 255, 0); //输入的初始点云相关
        viewer->addPointCloud(inputCloudI, color_handler, "cloud");
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            viewer2->removePointCloud("cloud");
            std::cout << "cluster size";
            pointProcessorI->numPoints(cluster);                                                                 // 控制台显示点云尺寸
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cluster, 255, 255, 0); //输入的初始点云相关
            viewer2->addPointCloud(cluster, color_handler, "cloud");
            // 显示每类障碍物点云
            // Fourth: Find bounding boxes for each obstacle cluster
            // 绘制边界框
            //os_Box(nh,cluster,clusterId);
            Box(cluster, clusterId, viewer);
            ++clusterId;
            //viewer->removeShape(std::to_string(1));
        }
        viewer->removePointCloud("cloud");
        viewer->removeAllShapes();
        viewer2->removeAllShapes();
    }

    PointCloudSegment::PointCloudSegment(ros::NodeHandle nh)
    {
        this->nodeHandle_ = nh;

        if (!readParameters())
        {
            ros::requestShutdown();
        }
        //ros_Box(nh);
        init();
    }

    PointCloudSegment::~PointCloudSegment() {}

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

        //初始化相机
        camera.init();

        //初始化viewer
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("All Cloud"));
        viewer2 = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Objects Only"));

        //声明一个XYZ类的点云处理对象
        ProcessPointClouds<pcl::PointXYZ> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZ>();

        Eigen::Vector3f eulerAngle;
        while (ros::ok())
        {
            //获取相机数据，包括点云指针和姿态角信息
            camera.updateNextFrame();
            inputCloudI = camera.getPointClouds();
            if (camera.getPose())
                eulerAngle = camera.getPose()->matrix().eulerAngles(2, 1, 0); //ZYX顺序(即RPY)的欧拉角

            // 当相机帧数据中有点云信息和姿态角信息时才进行检测
            if (inputCloudI && camera.getPose())
                cityBlock(pointProcessorI, inputCloudI, eulerAngle); //障碍物检测

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