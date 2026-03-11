#pragma once

#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pcl/common/transforms.h>

class LidarFilterNode : public rclcpp::Node
{
public:
  explicit LidarFilterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  struct ArenaWall
  {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
  };

  bool is_point_near_wall(double x, double y) const;
  sensor_msgs::msg::LaserScan::UniquePtr project_to_laser_scan(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laser_scan_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double wall_rejection_distance_;
  double ground_height_min_;
  double ground_height_max_;
  double laser_scan_min_angle_;
  double laser_scan_max_angle_;
  double laser_scan_angle_increment_;
  double laser_scan_range_min_;
  double laser_scan_range_max_;

  std::vector<ArenaWall> arena_walls_;
  std::string target_frame_;
  std::string source_frame_;
};
