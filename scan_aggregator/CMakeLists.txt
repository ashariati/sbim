cmake_minimum_required(VERSION 3.5)
project(scan_aggregator)

cmake_policy(SET CMP0144 NEW)
cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

add_executable(scan_aggregator src/scan_aggregator.cpp)
ament_target_dependencies(scan_aggregator
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        message_filters
        tf2
        tf2_eigen
        pcl_conversions
        pcl_msgs
)

install(TARGETS
        scan_aggregator
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()



