#include "pcl_seg/d435i.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr D435i::getNextFrame() try
{
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto color = frames.get_color_frame();

    // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
    if (!color)
        color = frames.get_infrared_frame();

    // Tell pointcloud object to map to this color frame
    pc.map_to(color);

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    return trans2pcl();
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
}

void D435i::init()
{
    // Start streaming with default recommended configuration
    pipe.start();
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
