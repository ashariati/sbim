cmake_minimum_required(VERSION 3.28.3)
project(scan_aggregator)

cmake_policy(SET CMP0144 NEW)
cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(scan_aggregator src/scan_aggregator.cpp)
ament_target_dependencies(scan_aggregator
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2
  tf2_eigen
  message_filters
)

target_link_libraries(scan_aggregator ${PCL_LIBRARIES})

ament_package()



