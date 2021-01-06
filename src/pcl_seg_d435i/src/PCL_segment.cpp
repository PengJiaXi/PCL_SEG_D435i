/*
 * PCL_segment.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Peng Jiaxi
 *   Institute: HUST, Intelligent Robotics Lab
 */
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

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
void PointCloudSegment::MyGetOBB (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ& min_point, pcl::PointXYZ& max_point, pcl::PointXYZ& position, Eigen::Matrix3f& rotational_matrix) const
    {
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);//质心点
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);//计算标准化协方差矩阵
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//pca特征向量
        Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//pca特征值
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
        /*
        // 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloudSegmented);
        pca.project(*cloudSegmented, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
        */
        Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();//齐次变换矩阵
        Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();//逆变矩阵
        tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();                                     //旋转矩阵R为特征向量的转置
        tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>()); //位移向量  -R*t
        tm_inv = tm.inverse();
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformedCloud, tm);//变换点云到pca特征向量确定的方向
        pcl::PointXYZ min_p1, max_p1; //变换后点云的最大值与最小值点
        Eigen::Vector3f c1, c;
        pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
        c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());//变换后下的质心
        Eigen::Affine3f tm_inv_aff(tm_inv);//逆变矩阵
        pcl::transformPoint(c1, c, tm_inv_aff);

        min_point = min_p1;
        max_point = max_p1;
        position.x = tm_inv(0, 3);
        position.y = tm_inv(1, 3);
        position.z = tm_inv(2, 3);
        rotational_matrix = tm_inv.block<3, 3>(0, 0);
    }
    void PointCloudSegment::ros_Box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int clusterId)
    {
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();
        //声明一些必要的变量
        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        //计算描述符和其他的特征
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        // feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        MyGetOBB (cloud,min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);    
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);
        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        // //中心的坐标
        // pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
        // pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
        // pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
        // pcl::PointXYZ z_ais (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
        // if (mass_center(2) < 1.0)
        {
            //画线
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "clouds_link_virtual";
            line_list.lifetime = ros::Duration(0.5);
            line_list.ns = "lines"; //+std::to_string(clusterId)
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.id = clusterId;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            line_list.scale.x = 0.01;
            // Line list is red
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;

            Eigen::Matrix<float, 3, 8> pv; //8个列向量组成矩阵，表示8个顶点坐标
            pv << min_point_OBB.x, min_point_OBB.x, max_point_OBB.x, max_point_OBB.x, min_point_OBB.x, min_point_OBB.x, max_point_OBB.x, max_point_OBB.x,
                min_point_OBB.y, min_point_OBB.y, min_point_OBB.y, min_point_OBB.y, max_point_OBB.y, max_point_OBB.y, max_point_OBB.y, max_point_OBB.y,
                min_point_OBB.z, max_point_OBB.z, max_point_OBB.z, min_point_OBB.z, min_point_OBB.z, max_point_OBB.z, max_point_OBB.z, min_point_OBB.z;
            Eigen::Matrix<float, 1, 8> one; //1*8的单位行向量
            one << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
            pv = rotational_matrix_OBB * pv + position * one; //计算旋转矩阵和质心平移后的真实坐标

            geometry_msgs::Point p[8]; //ros中Marker的8个顶点
            for (int i = 0; i < 8; i++)
            {
                p[i].x = pv(0, i);
                p[i].y = pv(1, i);
                p[i].z = pv(2, i);
            }
            const int order[24] = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 4, 0, 5, 1, 6, 2, 7, 3};
            for (int i = 0; i < 24; i++)
            {
                line_list.points.push_back(p[order[i]]);
            }
            line_list.header.stamp = ros::Time::now();
            boxPublisher_.publish(line_list);
            ros::spinOnce();
        }
    }
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

        // std::cout << "width1=" << whd1(0) << std::endl;
        // std::cout << "heght1=" << whd1(1) << std::endl;
        // std::cout << "depth1=" << whd1(2) << std::endl;
        // std::cout << "scale1=" << sc1 << std::endl;

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

        // 粗粒度参数
        float voxelsize = 0.04;
        // SegmentPlane
        int maxIterations = 100;
        float distanceThreshold = 0.03;
        // Clustering
        float clusterTolerance = 0.06;
        int minsize = 10;
        int maxsize = 1000;

        // 细粒度参数，必须固定相机
        // float voxelsize = 0.01;
        // // SegmentPlane
        // int maxIterations = 100;
        // float distanceThreshold = 0.01;
        // // Clustering
        // float clusterTolerance = 0.01;
        // int minsize = 4;
        // int maxsize = 1000;

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
        ground_point_cloud.header.frame_id = "clouds_link_virtual";
        groundPublisher_.publish(ground_point_cloud);

        // 发布障碍物点云信息
        sensor_msgs::PointCloud2 obstacle_point_cloud;
        pcl::toROSMsg(*segmentCloud.first, obstacle_point_cloud);
        obstacle_point_cloud.header.frame_id = "clouds_link_virtual";
        obstaclePublisher_.publish(obstacle_point_cloud);

        //原始点云
        sensor_msgs::PointCloud2 origin_point_cloud;
        pcl::toROSMsg(*inputCloudI, origin_point_cloud);
        origin_point_cloud.header.frame_id = "clouds_link_virtual";
        originPublisher_.publish(origin_point_cloud);

        //    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        //    renderPointCloud(viewer,inputCloud,"inputCloud");

        // Third: Cluster different obstacle cloud
        // 获得每个聚类的点云序列
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
                                                                                                              clusterTolerance,
                                                                                                              minsize, maxsize);

        //showCloudsInPCL(inputCloud, filteredCloud, segmentCloud, cloudClusters); //在PCL中显示点云
        showCloudsInROS(cloudClusters);
    }

    void PointCloudSegment::showCloudsInROS(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloudClusters)
    {
        int clusterId = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cluster, 255, 255, 0); //输入的初始点云相关
            // Fourth: Find bounding boxes for each obstacle cluster
            // 绘制边界框
            ros_Box(cluster, clusterId);
            ++clusterId;
        }
        // line_list.action=visualization_msgs::Marker::DELETEALL;
        // boxPublisher_.publish(line_list);
        // ros::spinOnce();
    }

    void PointCloudSegment::showCloudsInPCL(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &filteredCloud,
        const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> &segmentCloud,
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloudClusters)
    {
        //初始化viewer
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("All Cloud"));
        viewer2 = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Objects Only"));
        viewer3 = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Filted"));
        viewer4 = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Ground Plane"));

        int clusterId = 0;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(inputCloudI, 255, 255, 0); //输入的初始点云相关
        viewer->addPointCloud(inputCloudI, color_handler, "cloud");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler3(filteredCloud, 0, 255, 0);
        viewer3->addPointCloud(filteredCloud, color_handler3, "filter");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler4(segmentCloud.second, 0, 255, 255); //输入的初始点云相关
        viewer4->addPointCloud(segmentCloud.second, color_handler4, "ground");
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            viewer2->removePointCloud("cloud");
            //std::cout << "cluster size";
            //pointProcessorI->numPoints(cluster);                                                                 // 控制台显示点云尺寸
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cluster, 255, 255, 0); //输入的初始点云相关
            viewer2->addPointCloud(cluster, color_handler, "cloud");
            // 显示每类障碍物点云
            // Fourth: Find bounding boxes for each obstacle cluster
            // 绘制边界框
            // ros_Box(cluster,clusterId);
            Box(cluster, clusterId, viewer);
            ++clusterId;
        }
        viewer->removePointCloud("cloud");
        viewer->removeAllShapes();
        viewer2->removeAllShapes();
        viewer3->removeAllShapes();
        viewer3->removePointCloud("filter");
        viewer4->removeAllShapes();
        viewer4->removePointCloud("ground");
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
        imuPublisher_ =
            nodeHandle_.advertise<sensor_msgs::Imu>("imu_data", 1, false);
        boxPublisher_ =
            nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        originPublisher_ = 
            nodeHandle_.advertise<sensor_msgs::PointCloud2>("origin_points", 1, false);
        //初始化相机
        camera.init();

        //声明一个XYZ类的点云处理对象
        ProcessPointClouds<pcl::PointXYZ> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZ>();
        //rotation_estimator* temp;
        Eigen::Vector3f accelVector;
        sensor_msgs::Imu imu;
        while (ros::ok())
        {
            //获取相机数据，包括点云指针和姿态角信息
            camera.updateNextFrame();
            inputCloudI = camera.getPointClouds();
            accelVector = camera.getPose();

            // 发布IMU数据
            imu.header.stamp = ros::Time::now();
            imu.header.frame_id = "clouds_link_virtual";
            imu.linear_acceleration.x = accelVector.x();
            imu.linear_acceleration.y = accelVector.y();
            imu.linear_acceleration.z = accelVector.z();
            imu.orientation.x = 0;
            imu.orientation.y = 0;
            imu.orientation.z = 0;
            imu.orientation.w = 0;
            imu.angular_velocity.x = 0;
            imu.angular_velocity.y = 0;
            imu.angular_velocity.z = 0;
            imuPublisher_.publish(imu);

            cityBlock(pointProcessorI, inputCloudI, accelVector);
            // if(camera.isUpdated)       // 检测是否更新了点云数据
            // {
            //     camera.isUpdated = false;
            //     accelVector = camera.getPose(); //直接获取相机测得的线性加速度
            //     inputCloudI = camera.getPointClouds();
            //     cityBlock(pointProcessorI, inputCloudI, accelVector);
            // }

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