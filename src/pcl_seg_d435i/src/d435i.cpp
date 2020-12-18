#include "pcl_seg/d435i.h"

void D435i::updateNextFrame() try
{
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    auto color = frames.get_color_frame();

    // Cast the frame to pose_frame and get its data 
    if (f)
    {
        std::cout << "get pose frame" << std::endl;
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
        quaternion->w() = pose_data.rotation.w;
        quaternion->x() = pose_data.rotation.x;
        quaternion->y() = pose_data.rotation.y;
        quaternion->z() = pose_data.rotation.z;
    }
    else
        quaternion = NULL;

    pclptr = NULL;
    if (color)
    {
        std::cout << "get RGBD frame" << std::endl;
        // Tell pointcloud object to map to this color frame
        pc.map_to(color);
        auto depth = frames.get_depth_frame();
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        pclptr = trans2pcl();
    }
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
}

void D435i::init() try
{
    // 设置接收IMU信息和POSE
    cfg.enable_all_streams();
    // Start streaming with configuration
    pipe.start(cfg);
    std::cout << "D435i init" << std::endl;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    exit(-1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr D435i::getPointClouds()
{
    return pclptr;
}

Eigen::Quaternion<float>* D435i::getPose()
{
    return quaternion;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr D435i::trans2pcl()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}
