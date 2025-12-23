cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(unitree_lidar_sdk REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Find the static library
find_library(UNITREE_LIDAR_SDK_LIB
  NAMES libunitree_lidar_sdk.a
  HINTS ${unitree_lidar_sdk_DIR}/../../../lib
)

add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_link_libraries(unitree_lidar_ros2_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${UNITREE_LIDAR_SDK_LIB}
)

ament_target_dependencies(
  unitree_lidar_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  unitree_lidar_sdk
)

install(TARGETS
  unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# install rviz file and launch file
install(FILES rviz/view.rviz launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()