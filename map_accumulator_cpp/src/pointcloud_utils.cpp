#include "map_accumulator_cpp/pointcloud_utils.hpp"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace map_accumulator_cpp
{
namespace
{

int findFieldOffset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }

  throw std::runtime_error("Missing PointCloud2 field: " + name);
}

uint8_t findFieldDatatype(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return field.datatype;
    }
  }

  throw std::runtime_error("Missing PointCloud2 field: " + name);
}

float readFloat32(const uint8_t * ptr)
{
  float value;
  std::memcpy(&value, ptr, sizeof(float));
  return value;
}

uint32_t readUint32(const uint8_t * ptr)
{
  uint32_t value;
  std::memcpy(&value, ptr, sizeof(uint32_t));
  return value;
}

uint32_t readRgbBits(const uint8_t * ptr, uint8_t datatype)
{
  if (datatype == sensor_msgs::msg::PointField::UINT32) {
    return readUint32(ptr);
  }

  if (datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float rgb_float = readFloat32(ptr);
    uint32_t rgb_bits;
    std::memcpy(&rgb_bits, &rgb_float, sizeof(uint32_t));
    return rgb_bits;
  }

  throw std::runtime_error("Unsupported rgb field datatype in PointCloud2");
}

}  // namespace

CloudT::Ptr cloudFromRosMsg(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (msg.is_bigendian) {
    throw std::runtime_error("Big-endian PointCloud2 is not supported by this parser");
  }

  const int x_offset = findFieldOffset(msg, "x");
  const int y_offset = findFieldOffset(msg, "y");
  const int z_offset = findFieldOffset(msg, "z");
  const int rgb_offset = findFieldOffset(msg, "rgb");
  const uint8_t rgb_datatype = findFieldDatatype(msg, "rgb");

  auto cloud = std::make_shared<CloudT>();
  cloud->reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));

  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const std::size_t base =
        static_cast<std::size_t>(row) * msg.row_step +
        static_cast<std::size_t>(col) * msg.point_step;

      if (base + msg.point_step > msg.data.size()) {
        continue;
      }

      const float x = readFloat32(&msg.data[base + x_offset]);
      const float y = readFloat32(&msg.data[base + y_offset]);
      const float z = readFloat32(&msg.data[base + z_offset]);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const uint32_t rgb_bits = readRgbBits(&msg.data[base + rgb_offset], rgb_datatype);

      PointT point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.r = static_cast<uint8_t>((rgb_bits >> 16) & 0xFF);
      point.g = static_cast<uint8_t>((rgb_bits >> 8) & 0xFF);
      point.b = static_cast<uint8_t>(rgb_bits & 0xFF);

      cloud->push_back(point);
    }
  }

  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;

  return cloud;
}

sensor_msgs::msg::PointCloud2 cloudToRosMsg(
  const CloudT & cloud,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  return msg;
}

CloudT::Ptr transformCloud(
  const CloudT & input,
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;

  Eigen::Quaternionf quat(
    static_cast<float>(q.w),
    static_cast<float>(q.x),
    static_cast<float>(q.y),
    static_cast<float>(q.z));

  if (quat.norm() == 0.0f) {
    quat = Eigen::Quaternionf::Identity();
  } else {
    quat.normalize();
  }

  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  tf.block<3, 3>(0, 0) = quat.toRotationMatrix();
  tf(0, 3) = static_cast<float>(t.x);
  tf(1, 3) = static_cast<float>(t.y);
  tf(2, 3) = static_cast<float>(t.z);

  auto output = std::make_shared<CloudT>();
  pcl::transformPointCloud(input, *output, tf);

  return output;
}

CloudT::Ptr voxelDownsample(const CloudT & input, double voxel_size_m)
{
  auto output = std::make_shared<CloudT>();

  if (input.empty()) {
    return output;
  }

  if (voxel_size_m <= 0.0) {
    *output = input;
    return output;
  }

  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(input.makeShared());

  const float leaf_size = static_cast<float>(voxel_size_m);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*output);

  output->width = static_cast<uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;

  return output;
}

CloudT::Ptr limitCloud(const CloudT & input, std::size_t max_points)
{
  auto output = std::make_shared<CloudT>();

  if (max_points == 0 || input.size() <= max_points) {
    *output = input;
    return output;
  }

  output->reserve(max_points);

  const double step = static_cast<double>(input.size() - 1) /
                      static_cast<double>(max_points - 1);

  for (std::size_t i = 0; i < max_points; ++i) {
    const auto idx = static_cast<std::size_t>(std::round(i * step));
    output->push_back(input.points[idx]);
  }

  output->width = static_cast<uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;

  return output;
}

bool saveCloudPcdAndPly(
  const CloudT & cloud,
  const std::string & output_dir,
  const std::string & basename)
{
  if (cloud.empty()) {
    return false;
  }

  std::filesystem::create_directories(output_dir);

  const std::string pcd_path = output_dir + "/" + basename + ".pcd";
  const std::string ply_path = output_dir + "/" + basename + ".ply";

  const int pcd_result = pcl::io::savePCDFileBinary(pcd_path, cloud);
  const int ply_result = pcl::io::savePLYFileBinary(ply_path, cloud);

  return pcd_result == 0 && ply_result == 0;
}

}  // namespace map_accumulator_cpp
