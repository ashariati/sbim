cmake_minimum_required(VERSION 3.28.3)
project(tf_publishers)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(compass_to_tf src/compass_to_tf.cpp)
ament_target_dependencies(compass_to_tf rclcpp tf2_ros tf2_eigen geometry_msgs message_filters Eigen3)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp tf2_ros nav_msgs geometry_msgs)

add_executable(pose_to_tf src/pose_to_tf.cpp)
ament_target_dependencies(pose_to_tf rclcpp tf2_ros tf2_eigen geometry_msgs)

add_executable(transform_to_tf src/transform_to_tf.cpp)
ament_target_dependencies(transform_to_tf rclcpp tf2_ros tf2_eigen geometry_msgs Eigen3)

ament_package()
