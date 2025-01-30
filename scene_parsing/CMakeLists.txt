cmake_minimum_required(VERSION 3.28.3)
project(scene_parsing)

cmake_policy(SET CMP0144 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sbim_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

add_executable(cloud_transformer_node src/cloud_transformer_node.cpp)
ament_target_dependencies(cloud_transformer_node
    rclcpp
    sensor_msgs
    sbim_msgs
    pcl_conversions
    tf2_eigen
)

add_executable(door_detector_node src/door_detector_node.cpp)
target_include_directories(door_detector_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(door_detector_node
    rclcpp
    sensor_msgs
    sbim_msgs
    pcl_conversions
    tf2_eigen
    message_filters
)


add_executable(layout_extractor_node src/layout_extractor_node.cpp)
target_include_directories(layout_extractor_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(layout_extractor_node
    rclcpp
    sensor_msgs
    vision_msgs
    sbim_msgs
    pcl_conversions
    tf2_eigen
    message_filters
)

add_executable(plane_detector_node src/plane_detector_node.cpp)
target_include_directories(plane_detector_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(plane_detector_node
    rclcpp
    std_msgs
    sensor_msgs
    shape_msgs
    sbim_msgs
    pcl_conversions
    tf2_eigen
    message_filters
)


ament_package()

