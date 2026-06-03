#pragma once

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map_accumulator_cpp
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

CloudT::Ptr cloudFromRosMsg(const sensor_msgs::msg::PointCloud2 & msg);

sensor_msgs::msg::PointCloud2 cloudToRosMsg(
  const CloudT & cloud,
  const std_msgs::msg::Header & header);

CloudT::Ptr transformCloud(
  const CloudT & input,
  const geometry_msgs::msg::TransformStamped & transform);

CloudT::Ptr voxelDownsample(
  const CloudT & input,
  double voxel_size_m);

CloudT::Ptr limitCloud(
  const CloudT & input,
  std::size_t max_points);

bool saveCloudPcdAndPly(
  const CloudT & cloud,
  const std::string & output_dir,
  const std::string & basename);

}  // namespace map_accumulator_cpp
