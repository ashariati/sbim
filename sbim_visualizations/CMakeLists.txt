cmake_minimum_required(VERSION 3.28.3)
project(sbim_visualizations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sbim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(door_visualization src/door_visualization.cpp)
target_include_directories(door_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(door_visualization rclcpp visualization_msgs sbim_msgs)

add_executable(layout_segment_visualization src/layout_segment_visualization.cpp)
target_include_directories(layout_segment_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(layout_segment_visualization rclcpp visualization_msgs sbim_msgs)

add_executable(floorplan_visualization src/floorplan_visualization.cpp)
target_include_directories(floorplan_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(floorplan_visualization rclcpp visualization_msgs sbim_msgs)

add_executable(principal_directions_visualization src/principal_directions_visualization.cpp)
target_include_directories(principal_directions_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(principal_directions_visualization rclcpp visualization_msgs sbim_msgs)

add_executable(principal_planes_visualization src/principal_planes_visualization.cpp)
target_include_directories(principal_planes_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(principal_planes_visualization rclcpp visualization_msgs sbim_msgs Eigen3)

add_executable(trajectory_visualization src/trajectory_visualization.cpp)
target_include_directories(trajectory_visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(trajectory_visualization rclcpp visualization_msgs sbim_msgs Eigen3)

ament_package()
