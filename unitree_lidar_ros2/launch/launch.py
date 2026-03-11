import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # Run unitree lidar
  node1 = Node(
    package='unitree_lidar_ros2',
    executable='unitree_lidar_ros2_node',
    name='unitree_lidar_ros2_node',
    output='screen',
    parameters= [
              {'port': '/dev/ttyUSB0'},
              {'rotate_yaw_bias': 0.0},
              {'range_scale': 0.001},
              {'range_bias': 0.0},
              {'range_max': 50.0},
              {'range_min': 0.0},
              {'cloud_frame': "unilidar_lidar"},
              {'cloud_topic': "unilidar/cloud"},
              {'cloud_scan_num': 18},
              {'imu_frame': "unilidar_imu"},
              {'imu_topic': "unilidar/imu"}]
  )

  node2 = Node(
    package='unitree_lidar_ros2',
    executable='lidar_filter_node',
    name='lidar_filter_node',
    output='screen',
    parameters=[
              {'wall_rejection_distance': 0.3},
              {'ground_height_min': -0.05},
              {'ground_height_max': 0.05},
              {'target_frame': 'map'},
              {'source_frame': 'unilidar_lidar'},
              {'laser_scan_min_angle': -3.14159},
              {'laser_scan_max_angle': 3.14159},
              {'laser_scan_angle_increment': 0.00872664626},
              {'laser_scan_range_min': 0.1},
              {'laser_scan_range_max': 50.0}]
  )

  # # Run Rviz
  # package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
  # rviz_config_file = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
  # print("rviz_config_file = " + rviz_config_file)
  # rviz_node = Node(
  #   package='rviz2',
  #   executable='rviz2',
  #   name='rviz2',
  #   arguments=['-d', rviz_config_file],
  #   output='log'
  # )
  # return LaunchDescription([node1, rviz_node])

  return LaunchDescription([node1])