#include <gtest/gtest.h>

#include "map_accumulator_cpp/pointcloud_utils.hpp"

TEST(PointCloudUtils, VoxelDownsampleMergesNearbyPoints)
{
  auto cloud = std::make_shared<map_accumulator_cpp::CloudT>();

  map_accumulator_cpp::PointT p1;
  p1.x = 0.00F;
  p1.y = 0.00F;
  p1.z = 1.00F;
  p1.r = 100;
  p1.g = 100;
  p1.b = 100;

  map_accumulator_cpp::PointT p2;
  p2.x = 0.01F;
  p2.y = 0.01F;
  p2.z = 1.01F;
  p2.r = 200;
  p2.g = 200;
  p2.b = 200;

  map_accumulator_cpp::PointT p3;
  p3.x = 0.20F;
  p3.y = 0.20F;
  p3.z = 1.20F;
  p3.r = 10;
  p3.g = 20;
  p3.b = 30;

  cloud->push_back(p1);
  cloud->push_back(p2);
  cloud->push_back(p3);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  auto filtered = map_accumulator_cpp::voxelDownsample(*cloud, 0.05);

  EXPECT_EQ(filtered->size(), 2U);
}

TEST(PointCloudUtils, LimitCloudKeepsRequestedCount)
{
  map_accumulator_cpp::CloudT cloud;

  for (int i = 0; i < 10; ++i) {
    map_accumulator_cpp::PointT p;
    p.x = static_cast<float>(i);
    p.y = 0.0F;
    p.z = 1.0F;
    cloud.push_back(p);
  }

  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;

  auto limited = map_accumulator_cpp::limitCloud(cloud, 4);

  EXPECT_EQ(limited->size(), 4U);
}
