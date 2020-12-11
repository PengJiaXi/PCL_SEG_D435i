/*
 * PCL_segment_node.cpp
 *
 *  Created on: Oct 28, 2020
 *      Author: Peng Jiaxi
 *   Institute: HUST, Intelligent Robotics Lab
 */

#include <ros/ros.h>
#include "pcl_seg/PCL_segment.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "please_config_at_launch");
  ros::NodeHandle nodeHandle("~");
  pcl_process::PointCloudSegment pointCloudSegment(nodeHandle);
  
  ros::spin();
  return 0;
}