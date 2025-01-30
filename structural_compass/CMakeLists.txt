cmake_minimum_required(VERSION 3.28.3)
project(structural_compass)

cmake_policy(SET CMP0144 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sbim_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions)

add_executable(point_cloud_compass_node src/point_cloud_compass_node.cpp)

target_include_directories(point_cloud_compass_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(point_cloud_compass_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    sbim_msgs
    message_filters
    tf2
    tf2_eigen
    pcl_conversions
)


ament_package()

