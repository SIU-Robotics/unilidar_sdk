#include "lidar_filter_node.h"

LidarFilterNode::LidarFilterNode(const rclcpp::NodeOptions &options)
    : Node("lidar_filter_node", options)
{
  declare_parameter<double>("wall_rejection_distance", 0.3);
  declare_parameter<double>("ground_height_min", -0.05);
  declare_parameter<double>("ground_height_max", 0.05);
  declare_parameter<std::string>("target_frame", "map");
  declare_parameter<std::string>("source_frame", "unilidar_lidar");
  declare_parameter<double>("laser_scan_min_angle", -M_PI);
  declare_parameter<double>("laser_scan_max_angle", M_PI);
  declare_parameter<double>("laser_scan_angle_increment", 0.00872664626);
  declare_parameter<double>("laser_scan_range_min", 0.1);
  declare_parameter<double>("laser_scan_range_max", 50.0);

  wall_rejection_distance_ = get_parameter("wall_rejection_distance").as_double();
  ground_height_min_ = get_parameter("ground_height_min").as_double();
  ground_height_max_ = get_parameter("ground_height_max").as_double();
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  laser_scan_min_angle_ = get_parameter("laser_scan_min_angle").as_double();
  laser_scan_max_angle_ = get_parameter("laser_scan_max_angle").as_double();
  laser_scan_angle_increment_ = get_parameter("laser_scan_angle_increment").as_double();
  laser_scan_range_min_ = get_parameter("laser_scan_range_min").as_double();
  laser_scan_range_max_ = get_parameter("laser_scan_range_max").as_double();

  ArenaWall default_wall;
  default_wall.x_min = 0.0;
  default_wall.x_max = 10.0;
  default_wall.y_min = 0.0;
  default_wall.y_max = 10.0;
  arena_walls_.push_back(default_wall);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "unilidar/cloud", 10,
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

  pub_filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "unilidar/filtered_cloud", 10);

  pub_laser_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "unilidar/scan", 10);

  RCLCPP_INFO(this->get_logger(), "Lidar Filter Node initialized");
  RCLCPP_INFO(this->get_logger(), "Wall rejection distance: %.2f m", wall_rejection_distance_);
  RCLCPP_INFO(this->get_logger(), "Ground height range: [%.2f, %.2f] m", ground_height_min_, ground_height_max_);
}

bool LidarFilterNode::is_point_near_wall(double x, double y) const
{
  for (const auto &wall : arena_walls_)
  {
    if (x >= (wall.x_min - wall_rejection_distance_) && x <= (wall.x_max + wall_rejection_distance_))
    {
      if (y <= (wall.y_min + wall_rejection_distance_) || y >= (wall.y_max - wall_rejection_distance_))
      {
        return true;
      }
    }
    if (y >= (wall.y_min - wall_rejection_distance_) && y <= (wall.y_max + wall_rejection_distance_))
    {
      if (x <= (wall.x_min + wall_rejection_distance_) || x >= (wall.x_max - wall_rejection_distance_))
      {
        return true;
      }
    }
  }
  return false;
}

sensor_msgs::msg::LaserScan::UniquePtr LidarFilterNode::project_to_laser_scan(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
{
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();

  scan->header.frame_id = target_frame_;
  scan->header.stamp = this->now();
  scan->angle_min = laser_scan_min_angle_;
  scan->angle_max = laser_scan_max_angle_;
  scan->angle_increment = laser_scan_angle_increment_;
  scan->time_increment = 0.0;
  scan->scan_time = 0.1;
  scan->range_min = laser_scan_range_min_;
  scan->range_max = laser_scan_range_max_;

  size_t num_ranges = static_cast<size_t>((laser_scan_max_angle_ - laser_scan_min_angle_) / laser_scan_angle_increment_);
  scan->ranges.assign(num_ranges, laser_scan_range_max_);

  for (const auto &point : cloud->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;

    double range = std::sqrt(point.x * point.x + point.y * point.y);
    double angle = std::atan2(point.y, point.x);

    if (range < laser_scan_range_min_ || range > laser_scan_range_max_)
      continue;

    if (angle < laser_scan_min_angle_ || angle > laser_scan_max_angle_)
      continue;

    size_t angle_idx = static_cast<size_t>((angle - laser_scan_min_angle_) / laser_scan_angle_increment_);
    if (angle_idx < scan->ranges.size())
    {
      if (range < scan->ranges[angle_idx])
      {
        scan->ranges[angle_idx] = range;
      }
    }
  }

  return scan;
}

void LidarFilterNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty())
    return;

  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform(
        target_frame_,
        msg->header.frame_id,
        rclcpp::Time(0),
        rclcpp::Duration::from_seconds(0.5));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF transform unavailable: %s", ex.what());
    return;
  }

  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  Eigen::Quaternionf q(
    transform_stamped.transform.rotation.w,
    transform_stamped.transform.rotation.x,
    transform_stamped.transform.rotation.y,
    transform_stamped.transform.rotation.z
  );

  Eigen::Matrix3f rotation = q.toRotationMatrix();
  transform_matrix.block<3,3>(0,0) = rotation;

  transform_matrix(0,3) = transform_stamped.transform.translation.x;
  transform_matrix(1,3) = transform_stamped.transform.translation.y;
  transform_matrix(2,3) = transform_stamped.transform.translation.z;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  for (const auto &point : transformed_cloud->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;

    if (point.z >= ground_height_min_ && point.z <= ground_height_max_)
      continue;

    if (is_point_near_wall(point.x, point.y))
      continue;

    filtered_cloud->points.push_back(point);
  }

  filtered_cloud->header.frame_id = target_frame_;
  filtered_cloud->header.stamp = cloud->header.stamp;

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_msg);
  filtered_msg.header.stamp = this->now();
  filtered_msg.header.frame_id = target_frame_;
  pub_filtered_cloud_->publish(filtered_msg);

  auto scan = project_to_laser_scan(filtered_cloud);
  if (scan)
  {
    pub_laser_scan_->publish(*scan);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(LidarFilterNode)
