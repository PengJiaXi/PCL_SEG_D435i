#include "pcl_seg/d435i.h"

D435i *D435i::static_class_ptr = nullptr;

void D435i::updateNextFrame()
try
{
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    //std::cout << frames.get_frame_number() << std::endl;
    // Get a frame from the pose stream
    //auto f = frames.first_or_default(RS2_STREAM_ACCEL);
    auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    rs2::motion_frame accel = fa.as<rs2::motion_frame>();
    if (accel)
    {
        accelVector.x() = accel.get_motion_data().x;
        accelVector.y() = accel.get_motion_data().y;
        accelVector.z() = accel.get_motion_data().z;
        //std::cout << "accel: " << "x:" << accelVector.x() << "y:" << accelVector.y() << "z:" << accelVector.z() << std::endl;
    }
    
    auto color = frames.get_color_frame();
    pclptr = NULL;
    if (color)
    {
        //std::cout << "get RGBD frame" << std::endl;
        // Tell pointcloud object to map to this color frame
        pc.map_to(color);
        auto depth = frames.get_depth_frame();
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        pclptr = _trans2pcl();
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

void D435i::static_pipeCb(rs2::frame frame)
{
    static_class_ptr->_pipeCallback(frame);
}

void D435i::_pipeCallback(rs2::frame frame)
{
    // Cast the frame that arrived to motion frame
    auto motion = frame.as<rs2::motion_frame>();
    auto color = frame.as<rs2::video_frame>();
    auto depth = frame.as<rs2::depth_frame>();

    // If casting succeeded and the arrived frame is from accelerometer stream
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        // Get accelerometer measures
        rs2_vector accel_data = motion.get_motion_data();
        accelVector.x() = accel_data.x;
        accelVector.y() = accel_data.y;
        accelVector.z() = accel_data.z;
        // std::cout << "accel: "
        //           << "x:" << accelVector.x() << "y:" << accelVector.y() << "z:" << accelVector.z() << std::endl;
        // Call function that computes the angle of motion based on the retrieved measures
        //theta=algo.process_accel(accel_data);
    }

    if (color)
    {
        std::cout << "get RGBD frame" << std::endl;
        // Tell pointcloud object to map to this color frame
        pc.map_to(color);
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        pclptr = _trans2pcl();
    }
}

void D435i::init()
try
{
    static_class_ptr = this; //初始化静态类指针
    isUpdated = false;
    // 设置接收IMU信息和POSE
    cfg.enable_all_streams();
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    rotation_estimator algo;
    float3 theta;
    // Start streaming with configuration
    // auto profile = pipe.start(cfg, static_pipeCb);
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

Eigen::Vector3f D435i::getPose()
{
    return accelVector;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr D435i::_trans2pcl()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto &p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    //isUpdated = true;
    return cloud;
}

void rotation_estimator::process_gyro(rs2_vector gyro_data, double ts)
{
    if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        firstGyro = false;
        last_ts_gyro = ts;
        return;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;

    // Initialize gyro_angle with data from gyro
    gyro_angle.x = gyro_data.x; // Pitch
    gyro_angle.y = gyro_data.y; // Yaw
    gyro_angle.z = gyro_data.z; // Roll

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;

    // Apply the calculated change of angle to the current angle (theta)
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
}

float3 rotation_estimator::process_accel(rs2_vector accel_data)
{
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;
    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

    // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
    if (firstAccel)
    {
        firstAccel = false;
        theta = accel_angle;
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
        theta.y = PI;
    }
    else
    {
        /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
        theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
        theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
    }
    return theta; // Returns the current rotation angle
}

bool rotation_estimator::check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;
                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}