#include <chrono>
#include <memory>
#include <string>

#include "map_accumulator_cpp/pointcloud_utils.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class MapAccumulatorNode : public rclcpp::Node
{
public:
  MapAccumulatorNode()
  : Node("map_accumulator_cpp")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_cloud_topic", "/rgbd/cloud_filtered");
    output_topic_ = this->declare_parameter<std::string>(
      "map_cloud_topic", "/map/cloud");
    target_frame_ = this->declare_parameter<std::string>(
      "target_frame", "world");

    map_voxel_size_m_ = this->declare_parameter<double>(
      "map_voxel_size_m", 0.05);
    accumulate_every_n_frames_ = this->declare_parameter<int>(
      "accumulate_every_n_frames", 3);
    publish_every_n_processed_frames_ = this->declare_parameter<int>(
      "publish_every_n_processed_frames", 2);
    max_map_points_ = this->declare_parameter<int>(
      "max_map_points", 150000);
    use_latest_tf_fallback_ = this->declare_parameter<bool>(
      "use_latest_tf_fallback", true);

    save_final_map_on_shutdown_ = this->declare_parameter<bool>(
      "save_final_map_on_shutdown", true);
    output_dir_ = this->declare_parameter<std::string>(
      "output_dir",
      "/home/sudha/ros2_rgbd_ws/src/ros2-rgbd-mapping-pipeline/outputs/maps");
    output_basename_ = this->declare_parameter<std::string>(
      "output_basename", "final_map_cpp");

    if (accumulate_every_n_frames_ < 1) {
      accumulate_every_n_frames_ = 1;
    }

    if (publish_every_n_processed_frames_ < 1) {
      publish_every_n_processed_frames_ = 1;
    }

    if (max_map_points_ < 1) {
      max_map_points_ = 1;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_cloud_ = std::make_shared<map_accumulator_cpp::CloudT>();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MapAccumulatorNode::onCloud, this, _1));

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::QoS(1).reliable());

    RCLCPP_INFO(this->get_logger(), "C++ map accumulator node started");
    RCLCPP_INFO(this->get_logger(), "Subscribing cloud: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing map:     %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target frame:       %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_voxel_size_m=%.3f", map_voxel_size_m_);
    RCLCPP_INFO(this->get_logger(), "accumulate_every_n_frames=%d", accumulate_every_n_frames_);
    RCLCPP_INFO(
      this->get_logger(),
      "publish_every_n_processed_frames=%d",
      publish_every_n_processed_frames_);
    RCLCPP_INFO(this->get_logger(), "max_map_points=%d", max_map_points_);
    RCLCPP_INFO(this->get_logger(), "use_latest_tf_fallback=%s", use_latest_tf_fallback_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "save_final_map_on_shutdown=%s", save_final_map_on_shutdown_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "output_dir=%s", output_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_basename=%s", output_basename_.c_str());
  }

  void saveFinalMap()
  {
    if (!save_final_map_on_shutdown_) {
      return;
    }

    if (!map_cloud_ || map_cloud_->empty()) {
      RCLCPP_WARN(this->get_logger(), "No accumulated map points to save.");
      return;
    }

    const bool ok = map_accumulator_cpp::saveCloudPcdAndPly(
      *map_cloud_,
      output_dir_,
      output_basename_);

    if (ok) {
      RCLCPP_INFO(
        this->get_logger(),
        "Saved final map: %s/%s.pcd and %s/%s.ply",
        output_dir_.c_str(),
        output_basename_.c_str(),
        output_dir_.c_str(),
        output_basename_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save final map.");
    }
  }

private:
  geometry_msgs::msg::TransformStamped lookupTransform(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      return tf_buffer_->lookupTransform(
        target_frame_,
        msg->header.frame_id,
        rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & exact_time_error) {
      if (!use_latest_tf_fallback_) {
        throw exact_time_error;
      }

      return tf_buffer_->lookupTransform(
        target_frame_,
        msg->header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(0.05));
    }
  }

  void publishMap(const builtin_interfaces::msg::Time & stamp)
  {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = target_frame_;

    auto map_msg = map_accumulator_cpp::cloudToRosMsg(*map_cloud_, header);
    map_pub_->publish(map_msg);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    received_frames_++;

    if ((received_frames_ - 1) % static_cast<std::size_t>(accumulate_every_n_frames_) != 0) {
      return;
    }

    const auto start = std::chrono::steady_clock::now();

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = lookupTransform(msg);
    } catch (const tf2::TransformException & ex) {
      if (received_frames_ % 30 == 1) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      }
      return;
    }

    map_accumulator_cpp::CloudT::Ptr input_cloud;

    try {
      input_cloud = map_accumulator_cpp::cloudFromRosMsg(*msg);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(this->get_logger(), "PointCloud2 parse failed: %s", ex.what());
      return;
    }

    if (!input_cloud || input_cloud->empty()) {
      return;
    }

    auto world_cloud = map_accumulator_cpp::transformCloud(*input_cloud, transform_msg);

    *map_cloud_ += *world_cloud;

    map_cloud_ = map_accumulator_cpp::voxelDownsample(*map_cloud_, map_voxel_size_m_);
    map_cloud_ = map_accumulator_cpp::limitCloud(
      *map_cloud_,
      static_cast<std::size_t>(max_map_points_));

    processed_frames_++;

    if (
      processed_frames_ == 1 ||
      processed_frames_ % static_cast<std::size_t>(publish_every_n_processed_frames_) == 0)
    {
      publishMap(msg->header.stamp);
    }

    const auto stop = std::chrono::steady_clock::now();
    const auto latency_ms =
      std::chrono::duration<double, std::milli>(stop - start).count();

    if (processed_frames_ % 10 == 1) {
      RCLCPP_INFO(
        this->get_logger(),
        "processed_frames=%zu received_frames=%zu input_points=%zu map_points=%zu latency_ms=%.2f",
        processed_frames_,
        received_frames_,
        input_cloud->size(),
        map_cloud_->size(),
        latency_ms);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string output_dir_;
  std::string output_basename_;

  double map_voxel_size_m_{0.05};
  int accumulate_every_n_frames_{3};
  int publish_every_n_processed_frames_{2};
  int max_map_points_{150000};
  bool use_latest_tf_fallback_{true};
  bool save_final_map_on_shutdown_{true};

  std::size_t received_frames_{0};
  std::size_t processed_frames_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  map_accumulator_cpp::CloudT::Ptr map_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapAccumulatorNode>();

  rclcpp::spin(node);

  node->saveFinalMap();

  rclcpp::shutdown();
  return 0;
}
